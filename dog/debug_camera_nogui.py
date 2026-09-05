"""摄像头调试 - 无GUI模式，保存帧到文件"""

import cv2
import numpy as np
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vision.letter_detect import LetterDetector
from vision.gauge_detect import GaugeDetector
from vision.digit_detect import DigitDetector

# 摄像头参数
CAMERA_INDEX = 0
PROC_WIDTH = 1280
PROC_HEIGHT = 720


def main():
    # 初始化摄像头
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROC_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROC_HEIGHT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_GAIN, 0)

    if not cap.isOpened():
        print("ERROR: Camera cannot be opened!")
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera: {actual_w}x{actual_h}")

    # 初始化识别器
    letter_detector = LetterDetector()
    gauge_detector = GaugeDetector()
    digit_detector = DigitDetector()

    print("Testing camera... capturing 5 frames")

    try:
        for i in range(5):
            ret, frame = cap.read()
            if not ret:
                print(f"Frame {i}: Failed to read")
                continue

            # 保存原始帧
            filename = f"debug_frame_{i}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Frame {i}: Saved to {filename}, shape={frame.shape}")

            # 测试字母识别
            letter = letter_detector.detect(frame)
            print(f"  Letter detection: {letter}")

            # 测试仪表盘识别
            gauge_result = gauge_detector.detect(frame)
            print(f"  Gauge detection: {gauge_result}")

            # 测试数字识别
            digit_result = digit_detector.detect(frame)
            print(f"  Digit detection: {digit_result}")

            time.sleep(0.5)

        print("\nTest complete! Check debug_frame_*.jpg files")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        cap.release()
        print("Camera released")


if __name__ == "__main__":
    main()
