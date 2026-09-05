"""从摄像头画面裁剪字母模板"""

import cv2
import numpy as np
import os
import sys


def extract_letter_template(letter, output_dir="templates"):
    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))

    if not cap.isOpened():
        print("ERROR: Camera cannot be opened!")
        return

    ret, img = cap.read()
    cap.release()

    if not ret or img is None:
        print("Failed to capture frame")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    print(f"Captured frame: {w}x{h}")

    # C: 大步往右
    cx, cy = 660, 365
    hw, hh = 50, 55

    x1, x2 = cx - hw, cx + hw
    y1, y2 = cy - hh, cy + hh

    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

    roi = gray[y1:y2, x1:x2]

    preview_path = os.path.join(output_dir, f"{letter}_preview.png")
    cv2.imwrite(preview_path, roi)

    _, binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel = np.ones((2, 2), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    output_path = os.path.join(output_dir, f"{letter}.png")
    cv2.imwrite(output_path, binary)
    print(f"Saved {letter}.png: {binary.shape[1]}x{binary.shape[0]} pixels")

    return binary


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 extract_template.py <letter>")
        sys.exit(1)

    letter = sys.argv[1].upper()
    extract_letter_template(letter)
