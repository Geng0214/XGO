"""通用摄像头流服务器 - 在树莓派上运行
通过TCP把摄像头画面发送到本机，支持叠加识别结果

用法：
  python3 cam_server.py                        # 默认纯摄像头
  python3 cam_server.py --recognizer gauge     # 叠加仪表盘识别
  python3 cam_server.py --recognizer digit     # 叠加数显表计识别
  python3 cam_server.py --recognizer helmet    # 叠加安全帽识别
  python3 cam_server.py --recognizer letter    # 字母识别
"""

import socket
import cv2
import numpy as np
import struct
import sys
import os
import time
import argparse
import threading
import shutil
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from recognizers import RECOGNIZERS

HOST = "10.30.142.194"
PORT = 9999
PROC_WIDTH = 1280
PROC_HEIGHT = 720
JPEG_QUALITY = 50


class RPiCamSource:
    """通过 rpicam-vid 读取树莓派 CSI 摄像头（ov5647 等 libcamera 驱动），
    输出 BGR 帧。OpenCV 的 V4L2 后端打不开 CSI 摄像头，必须走 libcamera。"""

    def __init__(self, width, height, fps=30):
        self.width = width
        self.height = height
        cmd = [
            "rpicam-vid",
            "--inline",              # 输出流中保留 JPEG 起始/结束标记，便于切帧
            "--nopreview",
            "-t", "0",               # 无限时长
            "--width", str(width),
            "--height", str(height),
            "--framerate", str(fps),
            "--codec", "mjpeg",
            "--output", "-",
        ]
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.buf = b""

    def read(self):
        """阻塞直到返回一帧 BGR 图像；流结束返回 None。"""
        while True:
            start = self.buf.find(b"\xff\xd8")          # JPEG SOI
            if start == -1:
                chunk = self.proc.stdout.read(65536)
                if not chunk:
                    return None
                self.buf += chunk
                # 防止在首个 SOI 之前堆积过多无效数据
                if len(self.buf) > 1 << 20:
                    self.buf = self.buf[-1 << 20:]
                continue
            end = self.buf.find(b"\xff\xd9", start + 2)  # JPEG EOI
            if end == -1:
                chunk = self.proc.stdout.read(65536)
                if not chunk:
                    return None
                self.buf += chunk
                continue
            jpeg = self.buf[start:end + 2]
            self.buf = self.buf[end + 2:]
            frame = cv2.imdecode(
                np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                return frame
            # 解码失败则继续找下一帧

    def close(self):
        try:
            self.proc.terminate()
            self.proc.wait(timeout=2)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass


class Cv2Source:
    """普通 USB 摄像头（无 rpicam-vid 时的回退方案）。"""

    def __init__(self, width, height):
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_GAIN, 0)
        if not self.cap.isOpened():
            raise RuntimeError("Camera cannot be opened")

    def read(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    def close(self):
        self.cap.release()


def open_camera(width, height):
    """创建摄像头源，返回带 read()/close() 的对象。"""
    rpicam = shutil.which("rpicam-vid") or shutil.which("libcamera-vid")
    if rpicam:
        print("Using rpicam-vid for CSI camera capture")
        return RPiCamSource(width, height)
    print("Using OpenCV VideoCapture (fallback)")
    return Cv2Source(width, height)


def main():
    parser = argparse.ArgumentParser(description="Camera stream server")
    parser.add_argument("--recognizer", "-r", choices=list(RECOGNIZERS.keys()),
                        default=None, help="Recognition module to use")
    parser.add_argument("--port", "-p", type=int, default=PORT, help="TCP port")
    parser.add_argument("--quality", "-q", type=int, default=JPEG_QUALITY,
                        help=f"JPEG quality (1-100, default {JPEG_QUALITY})")
    args = parser.parse_args()

    recognizer = None
    if args.recognizer:
        print(f"Loading recognizer: {args.recognizer}")
        recognizer = RECOGNIZERS[args.recognizer]()

    # 打开摄像头源（树莓派 CSI 用 rpicam-vid，普通 USB 回退 OpenCV）
    try:
        frames = open_camera(PROC_WIDTH, PROC_HEIGHT)
    except RuntimeError as e:
        print(f"ERROR: {e}")
        return
    print(f"Camera: {PROC_WIDTH}x{PROC_HEIGHT}, JPEG quality={args.quality}")

    # 探测一帧，确认摄像头真正能出帧（避免“连上了但没图像”）
    probe = frames.read()
    if probe is None:
        print("ERROR: Camera did not deliver any frame")
        frames.close()
        return
    print(f"First frame OK: shape={probe.shape}")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, args.port))
    server.listen(1)
    print(f"Server listening on {HOST}:{args.port}")
    print("Waiting for client...")

    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Client connected: {addr}")

    # --- 识别线程：独立读帧+识别，结果写入共享变量 ---
    display_frame = [None]  # 可变容器，避免锁竞争
    running = True

    def recognition_loop():
        while running:
            frame = frames.read()
            if frame is None:
                break
            display_frame[0] = recognizer.process(frame)

    rec_thread = None
    if recognizer:
        rec_thread = threading.Thread(target=recognition_loop, daemon=True)
        rec_thread.start()

    fps_time = time.time()
    fps_count = 0
    last_result = None

    def send_frame(display):
        """叠加 FPS 并编码发送一帧"""
        nonlocal fps_count, fps_time
        fps_count += 1
        now = time.time()
        if now - fps_time >= 1.0:
            fps = fps_count / (now - fps_time)
            cv2.putText(display, f"FPS: {fps:.1f}", (PROC_WIDTH - 120, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            fps_count = 0
            fps_time = now
        _, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
        data = jpeg.tobytes()
        conn.sendall(struct.pack(">I", len(data)) + data)

    try:
        if recognizer:
            # 识别模式：取识别线程的最新结果
            while True:
                display = display_frame[0]
                if display is None:
                    time.sleep(0.005)
                    continue
                if args.recognizer == "letter":
                    letter = recognizer.detector.detect(display)
                    if letter != last_result:
                        print(f"Detected: {letter}")
                        last_result = letter
                send_frame(display)
        else:
            # 无识别器：主循环直接读帧发送
            while True:
                display = frames.read()
                if display is None:
                    time.sleep(0.01)
                    continue
                send_frame(display)

    except (BrokenPipeError, ConnectionResetError):
        print("Client disconnected")
    except KeyboardInterrupt:
        print("Server interrupted")
    finally:
        running = False
        if rec_thread:
            rec_thread.join(timeout=1)
        frames.close()
        conn.close()
        server.close()
        print("Server stopped")


if __name__ == "__main__":
    main()
