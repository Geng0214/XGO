#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红色小球识别 → 前后左右移动对准中心 → 靠近 → 机械臂夹取（精简版）
参考系统内置 /opt/luwu-os/libs/ball_catch_core.py 精简实现。

运行前先停 launcher 避免摄像头冲突：
    sudo systemctl stop luwu-launcher
运行：
    python3 ball.py        # 实时画面显示在机器人屏幕上，Ctrl+C 退出
"""
import os
import mmap
import time

import cv2
import numpy as np
from picamera2 import Picamera2
from xgolib import XGO

# ===================== 常量 =====================
CAM_W, CAM_H = 320, 240
CENTER_X = CAM_W // 2       # 画面中心 X
CENTER_Y = CAM_H // 2       # 画面中心 Y
EMA = 0.6                   # 检测结果平滑系数（偏快，减少靠近时半径滞后导致的过调振荡）
MIN_AREA = 50               # 红色区域最小面积(像素，过滤噪点)

CATCH_DIST = 22.0           # 抓取距离阈值(cm)
TARGET_DIST = 22.0          # 期望保持的距离(cm)
CENTER_TOL_X = 20           # 水平居中容差(像素)
CENTER_TOL_D = 3.0          # 距离容差(cm)
DIST_K = 54.82              # 距离-半径线性映射系数：distance = DIST_K - r（可按现场标定）

# ---- 步幅/速度说明 ----
# 底层速度上限: VX_LIMIT=25(move_x) / VY_LIMIT=18(move_y)。
# 参考 test.py 的四足控制思路：只要球未贴底就大步向前，只分快速/慢速两档。
# 踩坑: pace('high')+slow_trot 会原地高频踏步摇晃，且模式在固件跨运行持久化
#       → 启动时必须显式 pace('normal') 复位；大步幅靠 trot 步态+高限速实现。
PACE_MODE = 'normal'        # 踏步频率复位值（启动时显式复位；勿用 high+slow_trot）
APPROACH_GAIT = 'trot'      # 快速档步态（大步幅）
NEAR_GAIT = 'slow_trot'     # 慢速档步态（小步幅更稳）
MOVE_STEP = 0.4              # 每次移动指令保持时长(s)

LOST_MAX = 15               # 跟踪中连续丢球帧数：先停住，超时退回搜索
LOOP_SLEEP = 0.05           # 主循环间隔(s)

# ---- 搜索阶段（未检测到球时）----
SEARCH_SPEED = 8            # 搜索时持续慢速前进的速度（太低会原地踏步走不动）

# ---- 纵向(上下)位置分档：只分快速/慢速两档 ----
# 球放在地面：机器人越近，球心在画面中越靠下。
# 球未贴底（my 在 MY_NEAR 之上、离底部还有一段距离）→ 快速档大步向前；
# 球接近底部（my ≥ MY_NEAR）→ 慢速档小步精调。
MY_NEAR = 170              # 球心 y ≥ 该值 → 慢速档（已经很近）
MY_STOP = 200              # 球心 y ≥ 该值 → 贴底，停住并夹取

# 快速档：球未贴底，直接大步向前走（前进为主 + 横向修正）
FAST_GAIN_Y, FAST_GAIN_X = 0.20, 0.55
FAST_MIN_X = 15            # 快速档最低前进速度（保证大步向前）
FAST_MAX_Y, FAST_MAX_X = 14, 22
# 慢速档：球接近底部，小步精调（水平优先）
SLOW_GAIN_Y, SLOW_GAIN_X = 0.55, 0.66
SLOW_MAX_Y, SLOW_MAX_X = 6, 7
SLOW_TOL_X, SLOW_TOL_D = 10, 2.0

# ---- 目标夹取范围（微调第二阶段）：小球进入该范围 → 直接夹取 ----
# 夹取完全以球心 y（离底部距离）为准，不依赖半径（半径/EMA 滞后抖动会过早夹取）。
FINE_TUNE_RANGE = CATCH_DIST     # 距离进入该范围(cm)后进入慢速档
MY_GRAB = 193                # 球心 y ≥ 该值 → 真正贴近底部，直接夹取（MY_STOP=200 之前）
TARGET_TOL_X = 15            # 目标水平范围(像素)
SLOW_MIN_X = 2               # 慢速档小步前进速度（靠近底部抓取带）
TARGET_STABLE = 1            # 进入目标范围立即夹取
FINE_TUNE_TIMEOUT = 300      # 慢速档超时帧数，超时退回快速档

# 机器人屏幕（fbtft st7789v，320x240 RGB565 framebuffer）
FB_PATH = "/dev/fb0"
FB_W, FB_H = 320, 240

# 红色 HSV 区间(0-10 与 170-180 两段拼接)
RED_RANGES = [
    (np.array([0, 110, 70]), np.array([10, 255, 255])),
    (np.array([170, 110, 70]), np.array([180, 255, 255])),
]


# 步态缓存：只在步态变化时下发，避免反复下发导致机器狗原地踏步/抖动
_last_gait = [None]


def set_gait(dog, gait):
    """只在步态变化时下发 gait_type（静止/重复下发会原地踏步摇晃）"""
    if gait != _last_gait[0]:
        dog.gait_type(gait)
        _last_gait[0] = gait


# ---- 横移补偿参数（现场可调）----
SIDE_VY = 11            # 横移速度档（满速 18 太猛易斜，降速更稳）
SIDE_COMP_X = 2.5         # 横移时叠加的向前补偿速度（抵消 trot 横移向右后方跑偏）
SIDE_K = 0.0373         # 距离-时长系数（沿用 move_y_by 的标定值）


def side_step_right(dog, total_cm=80):
    """向右一次横移 total_cm cm（move_y 负值为向右），保持路线更直：
      横移同时叠加 SIDE_COMP_X 前进速度，抵消 trot 横移向右后方跑偏；
      速度用 SIDE_VY（低于满速 18，更稳）。
      k 值按满速 18 标定，这里按实际速度折算时长，确保横移走满 total_cm；
      横移结束后等待完全停稳再返回，保证“横移结束才进入找球”。
      现场微调：仍向后偏就加大 SIDE_COMP_X，仍斜就降低 SIDE_VY。
    """
    set_gait(dog, APPROACH_GAIT)
    # 时长按速度档折算：SIDE_K 是 vy=18 满速标定值，降速后按 18/SIDE_VY 比例放大，
    # 否则横移距离不足就走完，看起来像“还没横移完就开始找球”。
    runtime = SIDE_K * total_cm * (18.0 / SIDE_VY) + 0.5
    dog.move_y(-SIDE_VY)          # 负值 = 向右
    dog.move_x(SIDE_COMP_X)       # 向前补偿，抵消向后跑偏
    time.sleep(runtime)
    dog.move_y(0)
    dog.move_x(0)
    time.sleep(0.5)               # 等横移完全停稳再返回，随后才进入搜索


def _ball_from_contour(frame, mask, contour):
    """把一个候选红色轮廓处理成小球 (cx, cy, r)；不是圆形则返回 None。

    改进点：
      - 中心用轮廓质心（比 Hough/外接圆中心更贴近真实球心，位置更准）；
      - 半径优先用 Hough 圆（更准），失败用外接圆并收缩 12% 补偿边缘羽化；
      - 增加圆形度校验，过滤非圆形的红色杂色干扰，提高准确率。
    """
    # 圆形度校验：轮廓面积 vs 外接圆面积，太不圆的红色块多半是杂色干扰。
    # 圆形≈0.9+，矩形≈0.6、细长条≈<0.5；0.50 可兼顾“过滤杂色”与“不误杀半遮挡球”。
    (bx, by), br = cv2.minEnclosingCircle(contour)
    area = cv2.contourArea(contour)
    circ = area / (np.pi * br * br) if br > 0 else 0.0
    if circ < 0.50:
        return None

    # 球心：用轮廓质心（抗噪，比外接圆/检测圆中心更稳）
    M = cv2.moments(contour)
    cx = M["m10"] / M["m00"] if M["m00"] > 0 else bx
    cy = M["m01"] / M["m00"] if M["m00"] > 0 else by

    # 半径：优先 Hough 圆（半径更稳，避免屏幕有圈却说“未检出圆”）；
    # 失败则用外接圆收缩补偿（已验证屏幕上的圈位置准确）。
    # 关键：用外接圆半径做 Hough 搜索范围参考，显著提高检出率。
    filtered = np.zeros_like(mask)
    cv2.drawContours(filtered, [contour], -1, 255, -1)
    masked = cv2.bitwise_and(frame, frame, mask=filtered)
    gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)          # 平滑噪声利于 Hough
    lo_r = max(5, int(br * 0.6))                       # Hough 半径下界
    hi_r = max(lo_r + 1, min(50, int(br * 1.5)))       # Hough 半径上界
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2,
                               max(10, int(br * 0.5)), param1=40, param2=9,
                               minRadius=lo_r, maxRadius=hi_r)
    r = int(br * 0.88)   # 默认：外接圆收缩补偿
    if circles is not None:
        circles = np.round(circles[0, :]).astype(int)
        best = circles[np.argmax(circles[:, 2])]
        # 只接受圆心接近质心、半径在合理范围的圆
        if (abs(best[0] - cx) < 18 and abs(best[1] - cy) < 18
                and lo_r <= best[2] <= hi_r):
            r = best[2]

    return int(cx), int(cy), max(4, min(r, 45))


def detect_red_ball(frame):
    """检测画面中的红色小球，返回最左侧的 (cx, cy, r) 或 None。

    优先级逻辑：画面中同时出现多个球时，把每个通过圆形度校验的轮廓
    都处理成小球，然后优先返回最左边的球（cx 最小），便于从左往右
    依次夹取；画面中只有一个球时行为与原来一致。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in RED_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))

    # 开运算去噪 + 闭运算补洞；不膨胀，避免红色区域/半径被人为放大
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 轮廓 + 面积过滤，去掉细小噪点
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]
    if not valid:
        return None

    # 把所有通过圆形度校验的轮廓都处理成球，然后取最左边的优先夹取
    balls = [b for b in (_ball_from_contour(frame, mask, c) for c in valid) if b is not None]
    if not balls:
        return None
    balls.sort(key=lambda b: b[0])      # 按球心 x 升序：最左在前
    return balls[0]


def draw_ball(frame, ball, state="APPROACH"):
    """叠加检测结果（绿圈+十字+半径/偏移/状态）并返回用于显示的图像"""
    if ball is not None:
        x, y, r = ball
        cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
        cv2.line(frame, (x - r // 2, y), (x + r // 2, y), (0, 255, 0), 1)
        cv2.line(frame, (x, y - r // 2), (x, y + r // 2), (0, 255, 0), 1)
        cv2.putText(frame, f"r={r} ex={x - CENTER_X} ey={y - CENTER_Y} {state}",
                    (x - 70, y - r - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    else:
        cv2.putText(frame, f"SEARCH {state}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return frame


def open_screen():
    """打开机器人屏幕 framebuffer，返回 (fd, mmap缓冲)。屏幕为 320x240 RGB565。"""
    fd = os.open(FB_PATH, os.O_RDWR)
    return fd, mmap.mmap(fd, FB_W * FB_H * 2, mmap.MAP_SHARED)


def draw_screen(fbuf, frame_bgr):
    """把 BGR 帧写入屏幕 framebuffer（RGB565，小端字节序）"""
    # 与官方 ball_catch 一致：picamera2 的 "RGB888" 实为 BGR → 转 RGB → 水平翻转
    rgb = cv2.flip(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB), 1)
    r = (rgb[..., 0].astype(np.uint16) >> 3)
    g = (rgb[..., 1].astype(np.uint16) >> 2)
    b = (rgb[..., 2].astype(np.uint16) >> 3)
    rgb565 = (r << 11) | (g << 5) | b
    fbuf[:] = rgb565.astype("<u2").tobytes()


def control_move(dog, mx, my, mr, slow=False):
    """两档速度的移动控制（参考 test.py：球未贴底就大步向前）。

    返回 (distance, err_x, err_d, too_low)：
      - slow=False 快速档：球未贴底 → 前进为主大步向前 + 横向修正（斜向走）；
      - slow=True  慢速档：球接近底部 → 小步精调，水平优先；
      - too_low=True 表示球心已接近画面底部（太近），应立即停下夹取。
    """
    err_x = mx - CENTER_X                # 水平偏移(像素)
    distance = DIST_K - mr               # 按半径估算距离(cm)
    err_d = distance - TARGET_DIST       # 距离误差：>0 太远
    too_low = my >= MY_STOP              # 球心接近画面底部 → 太近

    # 两档：快速档大步向前 / 慢速档小步精调
    if slow:
        g_y, g_x, m_y, m_x = (SLOW_GAIN_Y, SLOW_GAIN_X,
                              SLOW_MAX_Y, SLOW_MAX_X)
        tol_x, tol_d = SLOW_TOL_X, SLOW_TOL_D
        gait = NEAR_GAIT
    else:
        g_y, g_x, m_y, m_x = (FAST_GAIN_Y, FAST_GAIN_X,
                              FAST_MAX_Y, FAST_MAX_X)
        tol_x, tol_d = CENTER_TOL_X, CENTER_TOL_D
        gait = APPROACH_GAIT

    if too_low:
        # 贴底：绝不再前进，只保持水平对准
        speed_y = int(np.clip(-g_y * err_x, -m_y, m_y))
        if speed_y == 0 and abs(err_x) > tol_x:
            speed_y = 1 if err_x < 0 else -1
        speed_x = 0
    elif not slow:
        # 快速档：大步向前（前进为主 + 横向修正防跑偏）
        speed_x = int(np.clip(g_x * err_d, -m_x, m_x))
        if abs(err_d) < tol_d:
            speed_x = 0
        elif err_d > 0 and speed_x < FAST_MIN_X:
            speed_x = FAST_MIN_X              # 保证大步向前
        speed_y = int(np.clip(-g_y * err_x, -m_y, m_y))
        if abs(err_x) < tol_x:
            speed_y = 0
        elif speed_y == 0:
            speed_y = 1 if err_x < 0 else -1
    else:
        # 慢速档：水平优先（未居中只左右，不前进）；
        # 前进以球心 y 为准，小步靠近底部抓取带（my>=MY_GRAB 后停住等夹取）
        if abs(err_x) > tol_x:
            speed_y = int(np.clip(-g_y * err_x, -m_y, m_y))
            if speed_y == 0:
                speed_y = 1 if err_x < 0 else -1   # 至少移动 1 档
            speed_x = 0
        else:
            speed_y = 0
            speed_x = SLOW_MIN_X if my < MY_GRAB else 0

    if speed_x or speed_y:
        set_gait(dog, gait)
        dog.move_y(speed_y)
        dog.move_x(speed_x)
        time.sleep(MOVE_STEP)
        dog.move_y(0)
        dog.move_x(0)
    else:
        time.sleep(LOOP_SLEEP)

    return distance, err_x, err_d, too_low


def do_catch(dog):
    """夹取序列：张开 → 伸出 → 闭合 → 收回"""
    dog.translation('z', 10)
    dog.attitude('p', 15)
    dog.claw(0)                  # 夹爪张开
    time.sleep(1)
    dog.arm_polar(200, 130)      # 伸出机械臂
    time.sleep(2)
    dog.claw(245)                # 闭合夹爪
    time.sleep(1)
    dog.arm_polar(90, 100)       # 收回机械臂
    time.sleep(1)
    print("[OK] 夹取完成")


# ---- 偏航角锁定/回正（参考 test.py 的闭环转向）----
ANGLE_TOLERANCE = 1.5   # 角度死区：误差小于该值即认为到位(°)
MIN_TURN_OUTPUT = 8     # 最小输出阈值：PID 输出小于该值时不下发，防止碎步漂移
TURN_KP, TURN_KI, TURN_KD = 1.2, 0.05, 0.5   # PID 参数（沿用 test.py）
TURN_MAX_OUT = 50       # 最大转向输出
TURN_TIMEOUT = 8.0      # 回正超时(s)
CONTROL_FREQ = 0.04     # 控制周期(s) = 25Hz
ARRIVAL_COUNT = 8       # 连续稳定在死区内的次数（约 0.3s）即认为到位


class IncrementalPID:
    """增量式 PID（来自 test.py）"""
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
        self.PIDOutput = max(min(self.PIDOutput, TURN_MAX_OUT), -TURN_MAX_OUT)
        self.LastLastError = self.LastError
        self.LastError = self.Error
        return int(self.PIDOutput)


def _norm_angle(a):
    """角度归一化到 [-180, 180]"""
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def _read_yaw_safe(dog, retries=3, delay=0.05):
    """安全读取 yaw：串口偶发超时/空读时自动重试，避免崩溃"""
    for _ in range(retries):
        try:
            return dog.read_yaw()
        except Exception:
            time.sleep(delay)
    return 0.0


def read_yaw_avg(dog, n=15, delay=0.02):
    """多次采样取平均，锁定当前航向（参考 test.py 的航向锁定）"""
    s = 0.0
    for _ in range(n):
        s += _read_yaw_safe(dog)
        time.sleep(delay)
    return _norm_angle(s / n)


def turn_to_yaw(dog, target_yaw, timeout=TURN_TIMEOUT):
    """闭环转向回正到目标偏航角（参考 test.py 的 turn_to_angle）：
    用 PID 输出连续修正，误差进入死区并连续稳定 ARRIVAL_COUNT 次后结束。
    """
    pid = IncrementalPID(TURN_KP, TURN_KI, TURN_KD)
    start_time = time.time()
    arrival_count = 0
    print(f"[转向] 回正到初始航向 {target_yaw:.1f}°")
    try:
        while time.time() - start_time < timeout:
            current_yaw = _read_yaw_safe(dog)
            correction = pid.Compute(target_yaw, current_yaw)

            if abs(pid.Error) < ANGLE_TOLERANCE:
                # 已进入误差死区：停住并累计稳定次数
                dog.turn(0)
                arrival_count += 1
            else:
                # 最小输出限制：小于阈值不下发，防止碎步漂移
                actual_turn = correction if abs(correction) >= MIN_TURN_OUTPUT else 0
                dog.turn(actual_turn)
                arrival_count = 0

            if arrival_count > ARRIVAL_COUNT:
                break
            time.sleep(CONTROL_FREQ)
    finally:
        dog.turn(0)
        dog.stop()
        time.sleep(0.5)   # 等完全站稳
    print(f"[转向] 回正完成，当前航向 {_read_yaw_safe(dog):.1f}°")


def main():
    dog = XGO("xgomini")
    
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (CAM_W, CAM_H)}
    )
    config["buffer_count"] = 1      # 只缓冲 1 帧：实时控制取最新帧，减少图像延迟
    picam2.configure(config)
    picam2.start()

    # 打开机器人屏幕 framebuffer（先黑屏）
    fb_fd, fbuf = open_screen()
    fbuf[:] = b"\x00" * (FB_W * FB_H * 2)

    # 锁定初始航向：在所有动作之前记录，供全部动作结束后回正用
    initial_yaw = read_yaw_avg(dog)
    print(f"[航向] 锁定初始航向: {initial_yaw:.2f}°")

    # 先向右横移 80cm，再压低身体进入抓球准备姿态。
    # 分步横移 + 前向补偿：避免 trot 横移向右后方跑偏，路线更直。
    side_step_right(dog, 80)
    print("[状态] 已向右横移 80cm，开始搜索")

    # 压低身体进入抓球准备姿态
    dog.attitude('p', 15)
    dog.translation('z', 75)
    time.sleep(1)

    # 初始停腿
    dog.move_x(0)
    dog.move_y(0)
    dog.turn(0)

    # 复位踏步频率为 normal（pace('high') 会在固件中跨运行持久化，
    # 且 +slow_trot 会原地高频踏步摇晃；步态只在移动时设置）
    dog.pace(PACE_MODE)
    time.sleep(0.5)      # 多等一会确保复位生效，避免开机就抖动

    mx = my = mr = 0.0   # EMA 平滑后的球心/半径
    lost = 0
    # 状态机：SEARCH 搜索 → APPROACH 快速档 → FINE_TUNE 慢速档 → CATCH 夹取
    state = "SEARCH"
    target_stable = 0    # 慢速档中连续在目标范围内的帧数
    fine_timeout = 0     # 慢速档超时计数（超时退回快速档）

    try:
        while True:
            frame = picam2.capture_array()
            if frame is None:
                continue

            ball = detect_red_ball(frame)

            # 实时显示画面（叠加检测结果与状态）到机器人屏幕
            draw_screen(fbuf, draw_ball(frame, ball, state))

            if state == "SEARCH":
                # 搜索：找到球 → 停住进入快速档；否则持续慢速前进（不走走停停，避免顿挫）
                if ball is not None:
                    dog.move_x(0)
                    dog.move_y(0)
                    lost = 0
                    state = "APPROACH"
                    print("[状态] 找到球，快速档大步接近")
                else:
                    set_gait(dog, NEAR_GAIT)
                    dog.move_x(SEARCH_SPEED)
                    time.sleep(LOOP_SLEEP)
                    continue

            if ball is None:
                # 跟踪中丢球：短时先停住等球，丢久了改为慢慢前进搜索（持续前进找球）。
                # 丢久后保持当前状态直接慢速前进，不退回 SEARCH，找到球即可继续对准。
                lost += 1
                if lost >= LOST_MAX:
                    set_gait(dog, NEAR_GAIT)
                    dog.move_x(SEARCH_SPEED)
                    time.sleep(LOOP_SLEEP)
                    continue
                else:
                    dog.move_x(0)
                    dog.move_y(0)
                    dog.turn(0)
                time.sleep(LOOP_SLEEP)
                continue

            # 平滑球心与半径
            lost = 0
            x, y, r = ball
            if mr == 0:
                mx, my, mr = x, y, r
            else:
                mx = EMA * x + (1 - EMA) * mx
                my = EMA * y + (1 - EMA) * my
                mr = EMA * r + (1 - EMA) * mr

            # 夹取：进入 CATCH 后执行夹取
            if state == "CATCH":
                dog.move_x(0)
                dog.move_y(0)
                dog.turn(0)
                do_catch(dog)               # 夹取序列（含收回机械臂）
                # 站立复位：站起并复位全部关节（含机械臂收回贴身），等完全站好后回正
                dog.action(2)               # 站起动作（官方 stand_up 方式）
                time.sleep(3)
                dog.reset()                 # 复位所有参数到初始状态
                time.sleep(1)
                turn_to_yaw(dog, initial_yaw)   # 回正到初始偏航角
                break

            distance, err_x, err_d, too_low = control_move(
                dog, mx, my, mr, slow=(state == "FINE_TUNE"))

            if state == "APPROACH":
                # 快速档：球贴底或已接近底部 → 停住切换
                if too_low:
                    # 贴底：先水平对准，对准后立即夹取（control_move 只左右不前进）
                    if abs(err_x) < CENTER_TOL_X:
                        dog.move_x(0)
                        dog.move_y(0)
                        state = "CATCH"
                        print(f"[状态] 球已贴底(y={my:.0f}) → 夹取")
                elif my >= MY_NEAR or distance <= FINE_TUNE_RANGE:
                    # 球接近底部 → 进入慢速档（微调第二阶段）
                    dog.move_x(0)
                    dog.move_y(0)
                    state = "FINE_TUNE"
                    target_stable = 0
                    fine_timeout = 0
                    print(f"[状态] 进入慢速档  距离={distance:.1f}cm  y={my:.0f}")
            elif state == "FINE_TUNE":
                if too_low:
                    # 慢速档中贴底 → 直接夹取
                    state = "CATCH"
                    print(f"[状态] 慢速档中球贴底(y={my:.0f}) → 夹取")
                elif my < MY_NEAR and distance > FINE_TUNE_RANGE + 5:
                    # 球跑远 → 退回快速档
                    state = "APPROACH"
                    target_stable = 0
                    print("[状态] 球跑远，退回快速档")
                else:
                    # 微调第二阶段：小球真正贴近底部（my≥MY_GRAB）且水平对准 → 直接夹取。
                    # 完全以球心 y 为准，不依赖半径（半径滞后会过早夹取）。
                    fine_timeout += 1
                    if abs(err_x) < TARGET_TOL_X and my >= MY_GRAB:
                        target_stable += 1
                        if target_stable >= TARGET_STABLE:
                            state = "CATCH"
                            print(f"[状态] 小球贴近底部(y={my:.0f})，直接夹取")
                    else:
                        target_stable = 0
                    if fine_timeout >= FINE_TUNE_TIMEOUT:
                        # 长时间没对准 → 退回快速档重试
                        state = "APPROACH"
                        target_stable = 0
                        fine_timeout = 0
                        print("[状态] 慢速档超时，退回快速档")

    except KeyboardInterrupt:
        print("\n手动退出")
    finally:
        dog.reset()
        picam2.stop()
        picam2.close()
        # 退出时黑屏并释放 framebuffer
        try:
            fbuf[:] = b"\x00" * (FB_W * FB_H * 2)
            fbuf.close()
            os.close(fb_fd)
        except Exception:
            pass


if __name__ == "__main__":
    main()
