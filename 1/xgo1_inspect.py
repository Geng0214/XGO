#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xgo1 区域巡检：A/B/C/D 四个区域横移到达 + 仪表盘状态播报
============================================================

功能（运动逻辑沿用 test.py）：
  1. 在下方【变量参数区】中选择任意两个字母（默认 A、C）
  2. 运动顺序：
       ① 前进 → ② 左转 180° → ③ 向左横移三次（前两次为 A、B 区域，
          第三次为额外横移）→ ④ 前进一段 → ⑤ 转向 180° 回到初始航向 →
          ⑥ 向左横移三次（前两次为 C、D 区域，第三次为额外横移）
       每次横移为 move_sideways(...)，结束后 time.sleep(0.5)
  3. 到达被选中字母的区域时，检测仪表盘并语音播报：
       - “X区域仪表盘显示偏高，状态异常”（指针指向红色区）
       - “X区域仪表盘显示正常，状态正常”（指针指向绿色区）
       - 未检出仪表盘/无摄像头时，播报固定文案“X区域仪表盘显示状态异常”

运行（依赖装在 .venv，务必用虚拟环境 python）：
    /home/luwu/WorkSpace/.venv/bin/python 1/xgo1_inspect.py
"""

import os
import sys
import threading
import time
from unittest.mock import MagicMock

import cv2
import numpy as np
from PIL import ImageFont
from xgolib import XGO

# =========================================================
# 1. 屏蔽屏幕硬件接管（防止 XGOEDU 强行接管屏幕导致黑屏）
sys.modules["xgoscreen"] = MagicMock()
sys.modules["xgoscreen.LCD_2inch"] = MagicMock()
# =========================================================
# 2. 兼容字体文件缺失问题（防止 /home/pi/model/msyh.ttc OSError）
_orig_truetype = ImageFont.truetype
def _patched_truetype(font, size=10, index=0, encoding='', *args, **kwargs):
    if isinstance(font, str) and not os.path.exists(font):
        candidates = [
            os.path.expanduser('~/model/msyh.ttc'),
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
            '/usr/share/fonts/truetype/freefont/FreeSans.ttf',
        ]
        for path in candidates:
            if os.path.exists(path):
                return _orig_truetype(path, size, index, encoding, *args, **kwargs)
        return ImageFont.load_default()
    return _orig_truetype(font, size, index, encoding, *args, **kwargs)
ImageFont.truetype = _patched_truetype
# =========================================================
# 3. 正常导入 xgoedu 库并初始化（用于多线程语音播报）
from xgoedu import XGOEDU
XGO_edu = XGOEDU()
# =========================================================

# ==========================================================
# ====================== 变量参数区 =========================
# ==========================================================
# 【在这里选择任意两个字母】
# 可选：'A'、'B'、'C'、'D'，任选两个。
# xgo1 会在对应区域播报“X区域仪表盘显示状态异常”
SELECTED_LETTERS = ["A", "C"]

# ---- 运动参数（沿用 test.py 的逻辑与数值）----
# 初始前进（① 前进）
FORWARD_SPEED = 20     # 前进速度
FORWARD_DURATION = 9   # 前进时长 (s)

# 中途前进（④ 在转向回初始航向之前前进一段）
MID_FORWARD_SPEED = 20
MID_FORWARD_DURATION = 7

# 横移参数（speed_y 正数向左）：每批三次横移 = 两次区域(A/B 或 C/D) + 一次额外
MOVE_STEP_1 = (10, 4)  # (速度, 时长) → A / C 区域（每批第一次）
MOVE_STEP_2 = (13, 5)  # (速度, 时长) → B / D 区域（每批第二次）
MOVE_EXTRA = (10, 4)   # (速度, 时长) → 额外横移（每批第三次，不播报）
MOVE_X_COMP = 2.4      # X 轴补偿（抵消横移时的前后漂移）

# 机器狗型号（按实际硬件修改）
DOG_TYPE = "xgomini"   # "xgomini" / "xgolite"

# 是否用机器狗内置 TTS 播报（True）；False 则只打印到终端
USE_BUILTIN_TTS = True
# ==========================================================
# ==========================================================


# ===================== 运动控制参数 =====================
KP = 1.2
KI = 0.05
KD = 0.5

ANGLE_TOLERANCE = 1.5   # 转向死区（°）
MIN_TURN_OUTPUT = 8     # 最小转向输出（电机死区之上）
CONTROL_FREQ = 0.04     # 25Hz
# ========================================================

dog = XGO(DOG_TYPE)


class IncrementalPID:
    """增量式 PID（用于前进/横移时锁定航向、转向回正）"""
    def __init__(self, P, I, D):
        self.Kp, self.Ki, self.Kd = P, I, D
        self.PIDOutput = 0.0
        self.Error = 0.0
        self.LastError = 0.0
        self.LastLastError = 0.0

    def Compute(self, target, current):
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        self.Error = error

        increment = (self.Kp * (self.Error - self.LastError) +
                     self.Ki * error +
                     self.Kd * (self.Error - 2 * self.LastError + self.LastLastError))
        self.PIDOutput += increment
        self.PIDOutput = max(min(self.PIDOutput, 50), -50)
        self.LastLastError = self.LastError
        self.LastError = self.Error
        return int(self.PIDOutput)


def lock_yaw():
    """多次采样取平均，锁定当前航向角"""
    sum_y = 0.0
    for _ in range(15):
        sum_y += dog.read_yaw()
        time.sleep(0.02)
    return sum_y / 15


def move_straight(speed, duration, target_yaw):
    """沿目标航向直线前进 duration 秒（PID 锁定航向）"""
    pid = IncrementalPID(KP, KI, KD)
    start_time = time.time()
    print(f"\n[前进] 目标航向: {target_yaw:.1f} | 速度: {speed} | 时长: {duration}s")
    try:
        while time.time() - start_time < duration:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)
            dog.move('x', int(speed))
            dog.turn(correction)
            time.sleep(CONTROL_FREQ)
    finally:
        dog.move('x', 0)
        dog.turn(0)
        time.sleep(0.3)
        dog.stop()


def turn_to_angle(target_yaw, timeout=8.0):
    """闭环转向到目标航向（误差死区 + 最小输出限制）"""
    pid = IncrementalPID(KP, KI, KD)
    start_time = time.time()
    arrival_count = 0
    print(f"\n[转向] 目标角度: {target_yaw:.1f}")
    try:
        while time.time() - start_time < timeout:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)

            if abs(pid.Error) < ANGLE_TOLERANCE:
                dog.move('x', 0)
                dog.turn(0)
                arrival_count += 1
            else:
                actual_turn = correction if abs(correction) >= MIN_TURN_OUTPUT else 0
                dog.move('x', 0)
                dog.turn(actual_turn)
                arrival_count = 0

            if arrival_count > 8:   # 连续 8 次（约 0.3s）稳定在死区内 → 完成
                break
            time.sleep(CONTROL_FREQ)
    finally:
        dog.move('x', 0)
        dog.turn(0)
        dog.stop()
        time.sleep(0.5)   # 等待物理稳定
        print(f"\n转向完成,最终角度: {dog.read_yaw():.1f}")


def move_sideways(speed_y, duration, target_yaw, x_compensation=MOVE_X_COMP):
    """
    横移（正数向左、负数向右），全程 PID 锁定航向。
    x_compensation: 横移时叠加的 X 轴补偿，抵消前后漂移。
    """
    pid = IncrementalPID(KP, KI, KD)
    start_time = time.time()
    print(f"\n[横移] 侧移速度: {speed_y} | 航向锁定: {target_yaw:.1f} | "
          f"X轴补偿: {x_compensation} | 时长: {duration}s")
    try:
        while time.time() - start_time < duration:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)
            dog.move('y', speed_y)          # 左右横移
            dog.move('x', x_compensation)   # 抵消前后漂移
            dog.turn(correction)            # 航向锁定
            time.sleep(CONTROL_FREQ)
    finally:
        dog.stop()


# ===================== A/B/C/D 四个变量 =====================
# A、B：第一批三次横移中的前两次到达的区域；
# C、D：第二批三次横移中的前两次到达的区域。
# 默认 False（未到达）；执行到对应横移后置 True。
A = False
B = False
C = False
D = False


def _visit(letter, params, target_yaw):
    """横移一次并到达 letter 区域：横移 → sleep(0.5) → 置位 → 若被选中则检测播报"""
    global A, B, C, D
    speed_y, duration = params
    print(f"\n===== 横移 → 到达 {letter} 区域 =====")
    move_sideways(speed_y=speed_y, duration=duration,
                  target_yaw=target_yaw, x_compensation=MOVE_X_COMP)
    time.sleep(0.5)   # 每次横移后停 0.5s

    # 置位对应字母变量（到达该区域）
    if letter == "A":
        A = True
    elif letter == "B":
        B = True
    elif letter == "C":
        C = True
    elif letter == "D":
        D = True

    # 被选中的区域：检测仪表盘并播报
    if letter in SELECTED_LETTERS:
        inspect_and_speak(letter)


def _slide(params, target_yaw, label="额外横移"):
    """额外横移一次（不关联 A/B/C/D，不播报）"""
    speed_y, duration = params
    print(f"\n===== {label}（速度 {speed_y} / 时长 {duration}s）=====")
    move_sideways(speed_y=speed_y, duration=duration,
                  target_yaw=target_yaw, x_compensation=MOVE_X_COMP)
    time.sleep(0.5)
# ===========================================================


# ===================== 语音播报（多线程，参照 voice.py） =====================
def text_to_speech_async(text):
    """后台线程语音播报：不阻塞主流程，巡检可继续进行"""
    def speech_worker():
        print("【后台线程】开始语音合成播报:", text)
        try:
            XGO_edu.SpeechSynthesis(text)
        except Exception as e:
            print(f"[警告] 语音播报失败({e})")
        print("【后台线程】语音播放完毕！")
    thread = threading.Thread(target=speech_worker, daemon=True)
    thread.start()
    return thread


def speak(text):
    """语音播报入口：按 voice.py 的方式，后台多线程异步播报"""
    print(f"[播报] {text}")
    if not USE_BUILTIN_TTS:
        return
    text_to_speech_async(text)
# =====================================================


# ===================== 仪表盘检测 =====================
# 比赛仪表盘布局：左=红(偏高)、上=绿(正常)、右下=黄(偏低)
GAUGE_RED_LOW = (0, 120, 70)
GAUGE_RED_HIGH = (10, 255, 255)
GAUGE_YELLOW_LOW = (15, 100, 70)
GAUGE_YELLOW_HIGH = (35, 255, 255)
GAUGE_GREEN_LOW = (35, 100, 70)
GAUGE_GREEN_HIGH = (85, 255, 255)


def _find_darkest_ray(gray, cx, cy, r):
    """从仪表盘中心向外发射射线，找最暗方向（黑色指针）的角度"""
    h, w = gray.shape
    best_angle, best_avg = None, 255
    for deg in range(0, 360, 2):
        rad = np.deg2rad(deg)
        vals = []
        for d in range(int(r * 0.15), int(r * 0.85), 5):
            px, py = int(cx + d * np.cos(rad)), int(cy + d * np.sin(rad))
            if 0 <= px < w and 0 <= py < h:
                vals.append(int(gray[py, px]))
        if vals:
            avg = sum(vals) / len(vals)
            if avg < best_avg:
                best_avg, best_angle = avg, deg
    return best_angle if best_avg < 80 else None


def _angle_to_zone(angle):
    """指针角度 → 颜色区域（0°=右，90°=下，180°=左，270°=上）"""
    if 135 <= angle <= 225:
        return "red"        # 左侧
    if 210 < angle < 330:
        return "yellow"     # 右下
    return "green"          # 上方


def _zone_to_status(zone):
    return {"red": "偏高", "yellow": "偏低", "green": "正常"}.get(zone, "未知")


def detect_gauge_status(frame):
    """识别仪表盘状态，返回 (zone, status) 或 None"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = gray.shape

    # 1. 找圆形仪表盘中心
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                               param1=80, param2=40, minRadius=30, maxRadius=150)
    if circles is not None:
        c = circles[0][0]
        cx, cy, r = int(c[0]), int(c[1]), int(c[2])
    else:
        cx, cy, r = w // 2, h // 2, min(w, h) // 3

    # 2. 射线扫描找指针方向
    angle = _find_darkest_ray(gray, cx, cy, r)
    if angle is not None:
        zone = _angle_to_zone(angle)
        return zone, _zone_to_status(zone)

    # 3. 回退：颜色面积法
    red = cv2.inRange(hsv, np.array(GAUGE_RED_LOW), np.array(GAUGE_RED_HIGH))
    yellow = cv2.inRange(hsv, np.array(GAUGE_YELLOW_LOW), np.array(GAUGE_YELLOW_HIGH))
    green = cv2.inRange(hsv, np.array(GAUGE_GREEN_LOW), np.array(GAUGE_GREEN_HIGH))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    areas = {
        "red": cv2.countNonZero(cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)),
        "yellow": cv2.countNonZero(cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kernel)),
        "green": cv2.countNonZero(cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)),
    }
    if sum(areas.values()) < 300:
        return None
    zone = max(areas, key=areas.get)
    return zone, _zone_to_status(zone)


def read_camera_frame():
    """读取一帧画面；摄像头打不开时返回 None（不影响巡检播报）"""
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        return frame if ret else None
    except Exception as e:
        print(f"[警告] 摄像头打开失败({e})")
        return None


def inspect_and_speak(letter):
    """在 letter 区域检测仪表盘并语音播报状态"""
    frame = read_camera_frame()
    result = detect_gauge_status(frame) if frame is not None else None
    if result:
        zone, status = result
        print(f"[识别] {letter}区域 zone={zone} status={status}")
        if status == "正常":
            text = f"{letter}区域仪表盘显示正常，状态正常"
        else:
            text = f"{letter}区域仪表盘显示{status}，状态异常"
    else:
        # 未检出仪表盘（或无摄像头）时，播报固定文案
        text = f"{letter}区域仪表盘显示状态异常"
    speak(text)
# =====================================================


# ===================== 主程序 =====================
def main():
    try:
        # 0. 复位 + 锁定初始航向
        dog.action(255)
        time.sleep(1)
        dog.pace('normal')
        initial_yaw = lock_yaw()
        print(f"航向锁定成功: {initial_yaw:.2f}")

        # ===================== 第一阶段（去程） =====================
        # 1. 前进
        move_straight(speed=FORWARD_SPEED, duration=FORWARD_DURATION,
                      target_yaw=initial_yaw)
        time.sleep(0.8)

        # 2. 左转 180°
        target_180 = initial_yaw + 180
        turn_to_angle(target_yaw=target_180)
        time.sleep(0.8)

        # 3. 向左横移三次：前两次为 A、B 区域，第三次为额外横移
        _visit("A", MOVE_STEP_1, target_180)   # 第一次横移 → A 区域
        _visit("B", MOVE_STEP_2, target_180)   # 第二次横移 → B 区域
        _slide(MOVE_EXTRA, target_180)         # 再横移一次

        # 4. 前进一段距离
        move_straight(speed=MID_FORWARD_SPEED, duration=MID_FORWARD_DURATION,
                      target_yaw=target_180)
        time.sleep(0.8)

        # 5. 转向 180° 回到初始航向
        turn_to_angle(target_yaw=initial_yaw)
        time.sleep(0.8)

        # ===================== 第二阶段（回程） =====================
        # 6. 向左横移三次：前两次为 C、D 区域，第三次为额外横移
        _visit("C", MOVE_STEP_1, initial_yaw)  # 第一次横移 → C 区域
        _visit("D", MOVE_STEP_2, initial_yaw)  # 第二次横移 → D 区域
        _slide(MOVE_EXTRA, initial_yaw)        # 最后再横移一次

        print("\n任务结束")
        print(f"到达状态: A={A} B={B} C={C} D={D}")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        dog.stop()


if __name__ == "__main__":
    main()
