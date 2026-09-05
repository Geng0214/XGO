"""通用摄像头流客户端 - 在本机上运行
接收树莓派的画面并显示

用法：
  python cam_client.py 192.168.0.43
  python cam_client.py 192.168.0.43 --port 9999
"""

import socket
import cv2
import numpy as np
import struct
import sys
import argparse
import threading


def recv_all(sock, n):
    """确保接收n个字节"""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 65536))
        if not chunk:
            raise ConnectionError("Server closed")
        buf += chunk
    return buf


def recv_loop(sock, frame_holder, running):
    """后台线程：持续接收帧并解码"""
    while running[0]:
        try:
            header = recv_all(sock, 4)
            data_len = struct.unpack(">I", header)[0]
            jpeg_data = recv_all(sock, data_len)
            frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                frame_holder[0] = frame
        except (ConnectionError, OSError):
            running[0] = False
            break


def main():
    parser = argparse.ArgumentParser(description="Camera stream client")
    parser.add_argument("host", help="Raspberry Pi IP address")
    parser.add_argument("--port", "-p", type=int, default=9999, help="TCP port")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
    print(f"Connecting to {args.host}:{args.port}...")
    sock.connect((args.host, args.port))
    print("Connected! Press 'q' to quit.")

    frame_holder = [None]
    running = [True]

    # 后台接收线程，解码和网络IO不阻塞显示
    t = threading.Thread(target=recv_loop, args=(sock, frame_holder, running), daemon=True)
    t.start()

    shown = 0
    stat_time = time.time()

    try:
        while running[0]:
            if frame_holder[0] is not None:
                cv2.imshow("Camera Stream", frame_holder[0])
                shown += 1
            now = time.time()
            if now - stat_time >= 2.0:
                if shown:
                    print(f"Displayed {shown} frames in last 2s", flush=True)
                else:
                    print("No frames received (check server / camera)", flush=True)
                shown = 0
                stat_time = now
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    finally:
        running[0] = False
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
