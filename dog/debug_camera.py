"""摄像头调试 - 实时读取帧并识别"""

import cv2
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vision.letter_detect import LetterDetector
from vision.gauge_detect import GaugeDetector
from vision.digit_detect import DigitDetector

# 摄像头参数 (参考 cam_server.py)
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

    print("Press 'q' to quit, 'g' for gauge, 'd' for digit, 'l' for letter")
    mode = "letter"  # 默认模式

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            display = frame.copy()

            # 根据模式进行识别
            if mode == "letter":
                letter = letter_detector.detect(frame)
                cv2.putText(display, f"Letter: {letter or 'None'}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            elif mode == "gauge":
                result = gauge_detector.detect(frame)
                if result:
                    cv2.putText(display, f"Gauge: {result['zone']} -> {result['status']}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(display, "Gauge: None", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            elif mode == "digit":
                result = digit_detector.detect(frame)
                if result:
                    cv2.putText(display, f"Digit: {result['value']} -> {result['status']}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(display, "Digit: None", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # 显示模式
            cv2.putText(display, f"Mode: {mode} (g/d/l to switch)",
                        (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("Debug Camera", display)

            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                mode = "gauge"
                print("Switched to gauge mode")
            elif key == ord('d'):
                mode = "digit"
                print("Switched to digit mode")
            elif key == ord('l'):
                mode = "letter"
                print("Switched to letter mode")

    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Done")


if __name__ == "__main__":
    main()
