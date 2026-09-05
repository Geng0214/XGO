"""仪表盘指针识别 - 射线扫描法"""

import cv2
import numpy as np
import math
from config import (GAUGE_RED_LOW, GAUGE_RED_HIGH,
                    GAUGE_YELLOW_LOW, GAUGE_YELLOW_HIGH,
                    GAUGE_GREEN_LOW, GAUGE_GREEN_HIGH)


class GaugeDetector:
    def __init__(self):
        pass

    def detect(self, frame):
        """
        识别仪表盘状态
        返回: {"zone": "red/yellow/green", "status": "偏高/偏低/正常", "angle": float} 或 None
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = gray.shape

        # 1. 找圆形仪表盘中心
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=80, param2=40, minRadius=30, maxRadius=150
        )
        if circles is not None:
            c = circles[0][0]
            cx, cy, r = int(c[0]), int(c[1]), int(c[2])
        else:
            cx, cy, r = w // 2, h // 2, min(w, h) // 3

        # 2. 从中心向外发射射线，找最暗方向（指针是黑色）
        needle_angle = self._find_darkest_ray(gray, cx, cy, r)

        if needle_angle is not None:
            zone = self._angle_to_zone(needle_angle)
            return {"zone": zone, "status": self._zone_to_status(zone), "angle": needle_angle}

        # 3. 回退：颜色面积法
        return self._detect_by_area(hsv)

    def _find_darkest_ray(self, gray, cx, cy, r):
        """从中心向外发射射线，返回最暗方向的角度(0-360)"""
        h, w = gray.shape
        best_angle = None
        best_avg = 255

        for deg in range(0, 360, 2):
            rad = math.radians(deg)
            vals = []
            for d in range(int(r * 0.15), int(r * 0.85), 5):
                px = int(cx + d * math.cos(rad))
                py = int(cy + d * math.sin(rad))
                if 0 <= px < w and 0 <= py < h:
                    vals.append(int(gray[py, px]))
            if vals:
                avg = sum(vals) / len(vals)
                if avg < best_avg:
                    best_avg = avg
                    best_angle = deg

        if best_avg < 80:
            return best_angle
        return None

    def _angle_to_zone(self, angle):
        """指针角度→颜色区域 (0°=右, 90°=下, 180°=左, 270°=上)
        比赛仪表盘布局：左=红(偏高), 上=绿(正常), 右下=黄(偏低)
        """
        if 135 <= angle <= 225:
            return "red"       # 左侧
        elif 210 < angle < 330:
            return "yellow"    # 右下
        else:
            return "green"     # 上方

    def _detect_by_area(self, hsv):
        """回退：颜色面积法"""
        red_mask = cv2.inRange(hsv, np.array(GAUGE_RED_LOW), np.array(GAUGE_RED_HIGH))
        yellow_mask = cv2.inRange(hsv, np.array(GAUGE_YELLOW_LOW), np.array(GAUGE_YELLOW_HIGH))
        green_mask = cv2.inRange(hsv, np.array(GAUGE_GREEN_LOW), np.array(GAUGE_GREEN_HIGH))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        areas = {
            "red": cv2.countNonZero(red_mask),
            "yellow": cv2.countNonZero(yellow_mask),
            "green": cv2.countNonZero(green_mask),
        }
        total = sum(areas.values())
        if total < 300:
            return None
        zone = max(areas, key=areas.get)
        return {"zone": zone, "status": self._zone_to_status(zone), "angle": -1}

    def _zone_to_status(self, zone):
        return {"red": "偏高", "yellow": "偏低", "green": "正常"}.get(zone, "未知")

    def get_debug_info(self, frame):
        """返回调试可视化"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        debug = np.zeros_like(frame)
        red_mask = cv2.inRange(hsv, np.array(GAUGE_RED_LOW), np.array(GAUGE_RED_HIGH))
        yellow_mask = cv2.inRange(hsv, np.array(GAUGE_YELLOW_LOW), np.array(GAUGE_YELLOW_HIGH))
        green_mask = cv2.inRange(hsv, np.array(GAUGE_GREEN_LOW), np.array(GAUGE_GREEN_HIGH))
        debug[:, :, 2] = red_mask
        debug[:, :, 1] = yellow_mask
        debug[:, :, 0] = green_mask

        overlay = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=80, param2=40, minRadius=30, maxRadius=150
        )
        if circles is not None:
            for c in circles[0]:
                cv2.circle(overlay, (int(c[0]), int(c[1])), int(c[2]), (0, 255, 0), 2)
        return overlay, debug
