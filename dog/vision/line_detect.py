"""黑线检测 + PID巡线"""

import cv2
import numpy as np
from config import BLACK_LINE_HSV_LOW, BLACK_LINE_HSV_HIGH


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error
        self.integral = max(-500, min(500, self.integral))
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.prev_error = 0
        self.integral = 0


class LineDetector:
    def __init__(self):
        pass

    def detect(self, frame):
        """返回黑线偏移量 (-1~1)，0=居中，None=未检测到"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(BLACK_LINE_HSV_LOW), np.array(BLACK_LINE_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 100:
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = M["m10"] / M["m00"]
        h, w = frame.shape[:2]
        offset = 2.0 * (cx - w / 2) / w
        return offset

    def get_mask(self, frame):
        """返回二值化mask用于调试显示"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(BLACK_LINE_HSV_LOW), np.array(BLACK_LINE_HSV_HIGH))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        return mask
