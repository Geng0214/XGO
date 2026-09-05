#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
安全帽检测 —— 无头识别入口（ultralytics YOLO 实现，不显示图像/屏幕）

模型：dog2/helmet/models/best.pt （YOLOv8，3 类: helmet1/nohelmet/helmet2）
  0=helmet1(戴帽)  1=nohelmet(未戴帽)  2=helmet2(戴帽)

用法：
  python3 run_helmet.py                            # 默认摄像头(0)
  python3 run_helmet.py --source 1                 # 指定摄像头索引
  python3 run_helmet.py --source rpi               # 树莓派 CSI 摄像头(rpicam-vid)
  python3 run_helmet.py --source video.mp4         # 视频文件
  python3 run_helmet.py --source photo.jpg         # 单张图片(检测后保存 result.jpg 退出)
  python3 run_helmet.py --conf 0.5 --target 3      # 调置信度/统计人数
  python3 run_helmet.py --model path/to/best.pt    # 指定模型
  python3 run_helmet.py --imgsz 320                # 推理分辨率(默认 640，调小可提速)
  python3 run_helmet.py --device cpu               # 推理设备(默认自动选择)

说明：无头模式，不弹窗也不写屏幕；检测/计数实时打印，达到目标人数后保存 result.jpg。
      Ctrl+C 结束。
"""

import os
import sys
import time
import shutil
import argparse
import subprocess

try:
    from ultralytics import YOLO
except ImportError:
    # 未激活 venv 时，自动切换到项目 .venv 重新执行本脚本
    _venv_python = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), ".venv", "bin", "python")
    if (os.path.exists(_venv_python)
            and os.path.abspath(sys.executable) != os.path.abspath(_venv_python)):
        os.execv(_venv_python, [_venv_python] + sys.argv)
    raise

import cv2
import numpy as np

DEFAULT_MODEL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "dog2", "helmet", "models", "best.pt")

HELMET_CLASSES = (0, 2)   # 戴帽类别: helmet1 / helmet2


def compute_iou(box_a, box_b):
    """两个 [x1,y1,x2,y2] 框的 IoU"""
    x1 = max(box_a[0], box_b[0]); y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2]); y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0


class PersonTracker:
    """IoU 多目标跟踪 + 去重统计（同一个 ID 只计一次）"""

    def __init__(self, iou_thresh=0.3, disappear_timeout=1.5):
        self.tracks = {}
        self.next_id = 0
        self.iou_thresh = iou_thresh
        self.disappear_timeout = disappear_timeout

    def update(self, detections, now):
        matched = []
        if detections:
            used_dets = set()
            used_tracks = set()
            iou_pairs = []
            for di, det in enumerate(detections):
                for tid, t in self.tracks.items():
                    iou = compute_iou(det["bbox"], t["box"])
                    if iou >= self.iou_thresh:
                        iou_pairs.append((iou, di, tid))
            iou_pairs.sort(key=lambda x: -x[0])
            for iou, di, tid in iou_pairs:
                if di in used_dets or tid in used_tracks:
                    continue
                det = detections[di]
                self.tracks[tid]["box"] = det["bbox"]
                self.tracks[tid]["helmet"] = det["helmet"]
                self.tracks[tid]["last_seen"] = now
                matched.append((tid, det))
                used_dets.add(di); used_tracks.add(tid)
            for di, det in enumerate(detections):
                if di in used_dets:
                    continue
                tid = self.next_id
                self.next_id += 1
                self.tracks[tid] = {"box": det["bbox"], "helmet": det["helmet"],
                                    "last_seen": now, "age": now}
                matched.append((tid, det))
        dead = [tid for tid, t in self.tracks.items()
                if now - t["last_seen"] > self.disappear_timeout]
        for tid in dead:
            del self.tracks[tid]
        return matched


class HelmetDetector:
    """安全帽检测器：ultralytics YOLO 推理 + 跟踪统计 + 画框"""

    def __init__(self, model_path=None, conf=0.85, target_count=5,
                 imgsz=640, device=None):
        if model_path is None:
            model_path = DEFAULT_MODEL
        self.conf = conf
        self.target_count = target_count
        self.imgsz = imgsz
        self.device = device
        self.model = YOLO(model_path)   # ultralytics 自动选择设备
        self.tracker = PersonTracker()
        self.seen_persons = {}      # tid -> {'helmet': bool}
        self.reached_target = False
        self.saved = False

    def detect(self, frame):
        """ultralytics 推理（自带 letterbox 预处理 + NMS），返回检测列表并更新跟踪统计"""
        results = self.model.predict(
            frame,
            conf=self.conf,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )
        detections = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                detections.append({
                    "bbox": [x1, y1, x2, y2],
                    "cls": cls,
                    "conf": conf,
                    "helmet": cls in HELMET_CLASSES,
                })

        now = time.time()
        matched = self.tracker.update(detections, now)
        for tid, det in matched:
            if tid not in self.seen_persons and not self.reached_target:
                self.seen_persons[tid] = {"helmet": det["helmet"]}
                status = "戴帽" if det["helmet"] else "未戴帽"
                print(f"[+] ID={tid} {status} 累计 {len(self.seen_persons)}/{self.target_count}")
                if len(self.seen_persons) >= self.target_count:
                    self.reached_target = True
                    print(f"*** 已达目标 {self.target_count} 人，停止计数 ***")
        return {"detections": detections,
                "helmet": sum(1 for p in self.seen_persons.values() if p["helmet"]),
                "no_helmet": sum(1 for p in self.seen_persons.values() if not p["helmet"])}

    def draw_detections(self, frame, detections):
        for tid, t in self.tracker.tracks.items():
            x1, y1, x2, y2 = [int(v) for v in t["box"]]
            color = (0, 255, 0) if t["helmet"] else (0, 0, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"ID{tid} {'Helmet' if t['helmet'] else 'No Helmet'}"
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        total = len(self.seen_persons)
        helmet = sum(1 for p in self.seen_persons.values() if p["helmet"])
        no_helmet = total - helmet
        cv2.putText(frame,
                    f"Total: {total}/{self.target_count}  Helmet: {helmet}  No Helmet: {no_helmet}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if self.reached_target and not self.saved:
            cv2.imwrite("result.jpg", frame)
            self.saved = True
        return frame

    def print_summary(self):
        total = len(self.seen_persons)
        helmet = sum(1 for p in self.seen_persons.values() if p["helmet"])
        no_helmet = total - helmet
        cn = {0: "零", 1: "一", 2: "二", 3: "三", 4: "四", 5: "五",
              6: "六", 7: "七", 8: "八", 9: "九", 10: "十"}
        print(f"\n【{cn.get(helmet, helmet)}人佩戴安全帽，{cn.get(no_helmet, no_helmet)}人没有佩戴安全帽】")


class RPiCamSource:
    """树莓派 CSI 摄像头(ov5647 等 libcamera 驱动)采集。
    OpenCV 的 V4L2 后端打不开 CSI 摄像头，必须走 rpicam-vid 子进程，
    按 JPEG 的 FFD8/FFD9 标记切帧。"""

    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        cam_cmd = shutil.which("rpicam-vid") or shutil.which("libcamera-vid")
        if cam_cmd is None:
            raise RuntimeError("未找到 rpicam-vid / libcamera-vid，无法采集 CSI 摄像头")
        cmd = [
            cam_cmd,
            "--inline",
            "--nopreview",
            "-t", "0",
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

    def release(self):
        try:
            self.proc.terminate()
        except Exception:
            pass


def _rpicam_cmd():
    """返回可用的 libcamera 视频采集命令名，没有则返回 None。"""
    return shutil.which("rpicam-vid") or shutil.which("libcamera-vid")


def _can_read_frame(cap, tries=3):
    """V4L2 能否真正读到帧（libcamera CSI 摄像头常出现能打开但读不到帧）。"""
    for _ in range(tries):
        ret, frame = cap.read()
        if ret and frame is not None:
            return True
    return False


def open_source(source):
    """返回 (cap, is_image)。source 支持数字、rpi、文件路径。"""
    if str(source).lower() == "rpi":
        return RPiCamSource(), False

    if isinstance(source, str) and os.path.isfile(source):
        ext = os.path.splitext(source)[1].lower()
        if ext in (".jpg", ".jpeg", ".png", ".bmp", ".webp"):
            return source, True          # 图片：直接返回路径
        cap = cv2.VideoCapture(source)  # 视频文件
        return cap, False

    idx = int(source)
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # libcamera CSI 摄像头(如树莓派 ov5647)：V4L2 打不开或读不到帧，自动改用 rpicam-vid
    readable = cap.isOpened() and _can_read_frame(cap)
    if not readable:
        cap.release()
        cam_cmd = _rpicam_cmd()
        if cam_cmd:
            print(f"摄像头 {idx} 无法通过 V4L2 读取帧，自动切换为 libcamera 采集 ({os.path.basename(cam_cmd)})")
            return RPiCamSource(), False
        print(f"警告: 摄像头 {idx} 无法通过 V4L2 读取帧，且未找到 rpicam-vid/libcamera-vid")
    return cap, False


def run_image(path, detector, args):
    """单张图片：检测一次，画框保存，打印摘要。"""
    frame = cv2.imread(path)
    if frame is None:
        print(f"无法读取图片: {path}")
        return 1

    print(f"图片 {path} 检测中...")
    result = detector.detect(frame)
    detector.draw_detections(frame, result["detections"])
    detector.print_summary()

    out = args.output or "result.jpg"
    cv2.imwrite(out, frame)
    print(f"结果已保存: {out}")
    return 0


def run_stream(cap, detector):
    """摄像头/视频：无头循环识别（检测+跟踪+统计，不显示画面）。"""
    frame_count = 0
    try:
        while True:
            if isinstance(cap, RPiCamSource):
                frame = cap.read()
            else:
                ret, frame = cap.read()
                if not ret:
                    print("画面结束")
                    break

            if frame is None:
                continue

            frame_count += 1
            if frame_count % 30 == 1:
                print(f"--- Frame {frame_count} ---")
            result = detector.detect(frame)
            detector.draw_detections(frame, result["detections"])
    finally:
        detector.print_summary()
        if hasattr(cap, "release"):
            cap.release()
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="安全帽检测无头识别入口（不显示图像/屏幕）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument("--source", default="0",
                        help="摄像头索引(0/1)、rpi、视频文件或图片路径 (默认 0)")
    parser.add_argument("--model", default=None,
                        help="模型路径 (默认 dog2/helmet/models/best.pt)")
    parser.add_argument("--conf", type=float, default=0.85,
                        help="检测置信度阈值 (默认 0.85)")
    parser.add_argument("--target", type=int, default=5,
                        help="需要统计的目标人数 (默认 5)")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="推理输入尺寸 (默认 640，调小可提速)")
    parser.add_argument("--device", default=None,
                        help="推理设备: cpu / 0 (默认自动选择)")
    parser.add_argument("--output", "-o", default=None,
                        help="图片模式下结果保存路径 (默认 result.jpg)")
    args = parser.parse_args()

    detector = HelmetDetector(
        model_path=args.model,
        conf=args.conf,
        target_count=args.target,
        imgsz=args.imgsz,
        device=args.device,
    )
    print(f"模型加载完成 | conf={args.conf} | 目标人数={args.target} | imgsz={args.imgsz}")

    cap, is_image = open_source(args.source)
    if is_image:
        return run_image(cap, detector, args)

    if not isinstance(cap, RPiCamSource) and not cap.isOpened():
        print(f"无法打开摄像头/视频: {args.source}")
        return 1

    print(f"来源: {args.source}")
    return run_stream(cap, detector)


if __name__ == "__main__":
    sys.exit(main())
