"""摄像头读取封装"""

import cv2
from config import CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT


class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        # 白平衡与色彩校正
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_GAIN, 0)

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        if len(frame.shape) == 2 and frame.shape[1] == CAMERA_WIDTH * CAMERA_HEIGHT:
            frame = frame.reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        return frame if len(frame.shape) == 3 else None

    def release(self):
        self.cap.release()
