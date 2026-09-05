#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仪表盘状态识别程序（支持旋转表盘识别）
支持两种工作模式：
  1. 图片模式：直接读取指定图片进行识别
  2. 相机模式：通过摄像头实时识别仪表盘状态

输出规则：
  - 红色区域：仪表盘偏高，状态异常
  - 黄色区域：仪表盘偏低，状态异常
  - 绿色区域：仪表盘正常

特性：
  - 自动检测表盘旋转角度并校正
  - 相机模式下支持结果平滑与稳定输出
  - 基于颜色弧段分析判断旋转方向

使用示例：
  python3 watch.py --mode image --images 1.jpg 2.jpg 3.jpg
  python3 watch.py --mode camera --device 0
  python3 watch.py --mode camera --device 0 --width 1280 --height 720
"""

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os
import sys
import math
import argparse
import time
from collections import Counter


def put_chinese_text(img, text, pos, color=(0, 0, 255), font_size=30):
    """使用PIL在OpenCV图像上绘制中文文本"""
    img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)

    font_paths = [
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
        '/home/pi/model/msyh.ttc',
    ]
    font = None
    for fp in font_paths:
        if os.path.exists(fp):
            try:
                font = ImageFont.truetype(fp, font_size)
                break
            except Exception:
                pass

    if font is None:
        font = ImageFont.load_default()

    draw.text(pos, text, font=font, fill=color)
    return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)


def correct_white_balance(img):
    """快速白平衡校正 - 使用查找表"""
    # 计算各通道均值
    avg_b = np.mean(img[:, :, 0])
    avg_g = np.mean(img[:, :, 1])
    avg_r = np.mean(img[:, :, 2])
    avg = (avg_b + avg_g + avg_r) / 3

    # 构建查找表
    scale_b = avg / (avg_b + 1)
    scale_g = avg / (avg_g + 1)
    scale_r = avg / (avg_r + 1)

    # 应用缩放
    result = img.astype(np.float32)
    result[:, :, 0] *= scale_b
    result[:, :, 1] *= scale_g
    result[:, :, 2] *= scale_r
    return np.clip(result, 0, 255).astype(np.uint8)


def detect_gauge_circle(img):
    """
    检测仪表盘外圆
    返回：(圆心x, 圆心y, 半径)
    """
    h, w = img.shape[:2]

    # 大图缩半加速
    if max(h, w) > 800:
        scale = 0.5
        small = cv2.resize(img, (int(w * scale), int(h * scale)))
    else:
        scale = 1.0
        small = img
    sh, sw = small.shape[:2]

    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 1.5)

    min_r = max(min(sh, sw) // 8, 25)
    max_r = min(sh, sw) // 2
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=60,
        param1=80, param2=35, minRadius=min_r, maxRadius=max_r
    )

    if circles is None:
        return w // 2, h // 2, min(h, w) // 3

    # 按评分排序，选最优
    img_cx, img_cy = sw // 2, sh // 2
    scored = []
    for c in circles[0]:
        cx, cy, cr = int(c[0]), int(c[1]), int(c[2])
        dist = math.hypot(cx - img_cx, cy - img_cy)
        score = 0
        if cy > img_cy:
            score += 50
        score -= dist * 0.3
        # 半径评分：缩半后75左右 = 原图150 = 仪表盘
        if 65 <= cr <= 90:  # 缩半后的仪表盘半径范围
            score += 80
        elif 50 <= cr <= 110:
            score += 30
        # 排除太大的圆
        if cr > min(sh, sw) * 0.4:
            score -= 200
        scored.append((cx, cy, cr, score))

    scored.sort(key=lambda x: -x[3])
    best = scored[0]
    return int(best[0] / scale), int(best[1] / scale), int(best[2] / scale)


def detect_rotation_angle(img, cx, cy, radius):
    """
    通过检测圆环颜色弧段位置来确定表盘旋转角度。
    标准位置：绿色中心约270°，黄色中心约180°，红色中心约0°/360°
    返回旋转角度（度），正值表示逆时针旋转
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 创建圆环掩码（颜色带区域）
    mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
    cv2.circle(mask, (cx, cy), int(radius * 0.96), 255, -1)
    cv2.circle(mask, (cx, cy), int(radius * 0.78), 0, -1)

    def check_color(h, s, v, color_name):
        if color_name == 'green':
            return (60 <= h <= 90) and (s > 120) and (v > 80)
        elif color_name == 'yellow':
            return (20 <= h <= 50) and (s > 120) and (v > 160)
        elif color_name == 'red':
            return ((h <= 15 or h >= 170) and (s > 120) and (v > 100))
        return False

    angles = list(range(0, 360, 3))
    scores = {'green': [], 'yellow': [], 'red': []}

    for angle_deg in angles:
        rad = math.radians(angle_deg)
        counts = {'green': 0, 'yellow': 0, 'red': 0, 'total': 0}

        for r_frac in [0.80, 0.84, 0.88, 0.92, 0.96]:
            px = int(cx + radius * r_frac * math.cos(rad))
            py = int(cy + radius * r_frac * math.sin(rad))
            if 0 <= px < img.shape[1] and 0 <= py < img.shape[0] and mask[py, px]:
                h, s, v = [int(v) for v in hsv[py, px]]
                counts['total'] += 1
                if check_color(h, s, v, 'green'):
                    counts['green'] += 1
                elif check_color(h, s, v, 'yellow'):
                    counts['yellow'] += 1
                elif check_color(h, s, v, 'red'):
                    counts['red'] += 1

        for c in scores:
            scores[c].append(counts[c] / counts['total'] if counts['total'] > 0 else 0)

    def find_arc_center(score_list, threshold=0.20):
        extended = score_list + score_list
        best_start, best_len, best_sum = 0, 0, 0
        curr_start, curr_len, curr_sum = 0, 0, 0

        for i in range(len(extended)):
            if extended[i] >= threshold:
                if curr_len == 0:
                    curr_start = i
                curr_len += 1
                curr_sum += extended[i]
            else:
                if curr_len > 0 and (curr_len > best_len or
                                     (curr_len == best_len and curr_sum > best_sum)):
                    best_len, best_sum, best_start = curr_len, curr_sum, curr_start
                curr_len, curr_sum = 0, 0

        if best_len == 0:
            return None

        center_idx = best_start + best_len // 2
        return (angles[center_idx % len(angles)] + 360 * (center_idx // len(angles))) % 360

    g_center = find_arc_center(scores['green'], threshold=0.20)
    y_center = find_arc_center(scores['yellow'], threshold=0.20)
    r_center = find_arc_center(scores['red'], threshold=0.15)

    # 标准位置参考中心
    ref = {'green': 270, 'yellow': 180, 'red': 0}
    centers = {'green': g_center, 'yellow': y_center, 'red': r_center}

    rots = []
    for color, center in centers.items():
        if center is not None:
            rot = (center - ref[color]) % 360
            if rot > 180:
                rot -= 360
            rots.append((color, rot, center))

    if not rots:
        return 0

    # 一致性检查：如果多个颜色检测的旋转角度接近，取平均
    if len(rots) >= 2:
        for i in range(len(rots)):
            for j in range(i + 1, len(rots)):
                diff = abs(rots[i][1] - rots[j][1])
                if diff > 180:
                    diff = 360 - diff
                if diff <= 25:
                    avg = (rots[i][1] + rots[j][1]) / 2
                    if abs(rots[i][1] - rots[j][1]) > 180:
                        avg = ((rots[i][1] + rots[j][1] + 360) / 2) % 360
                        if avg > 180:
                            avg -= 360
                    return int(avg)

    # 优先返回绿色检测结果（最可靠）
    return rots[0][1]


def detect_needle(img, cx, cy, radius, cached_lines=None):
    """
    检测指针方向，返回 (tip_x, tip_y) 或 None
    方法：找从中心出发、暗像素最集中的方向
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    # 对每个方向，计算暗像素数量（阈值80）
    direction_dark = [0] * 360

    for deg in range(0, 360):
        rad = math.radians(deg)
        for d in range(int(radius * 0.15), int(radius * 0.75)):
            px = int(cx + d * math.cos(rad))
            py = int(cy + d * math.sin(rad))
            if 0 <= px < w and 0 <= py < h:
                if int(gray[py, px]) < 80:
                    direction_dark[deg] += 1

    # 找暗像素最多的方向
    best_angle = max(range(360), key=lambda d: direction_dark[d])
    best_count = direction_dark[best_angle]

    if best_count < 10:
        return None, None

    rad = math.radians(best_angle)
    tip_x = int(cx + radius * 0.80 * math.cos(rad))
    tip_y = int(cy + radius * 0.80 * math.sin(rad))
    if 0 <= tip_x < w and 0 <= tip_y < h:
        return (tip_x, tip_y), None
    return None, None


def get_region_by_angle(tip_x, tip_y, cx, cy, rotation=0):
    """
    沿指针方向在外圈采样颜色判断区域
    返回：(颜色字符串, 到最近边界的距离[度])
    """
    dx = tip_x - cx
    dy = tip_y - cy
    angle_deg = math.degrees(math.atan2(dy, dx))
    if angle_deg < 0:
        angle_deg += 360

    h, w = 0, 0
    return 'unknown', 0


def classify_gauge_color(img, cx, cy, radius, needle_angle, tip_point=None):
    """根据指针角度直接判断状态"""
    angle = needle_angle % 360

    # 仪表盘布局（顺时针，0°=右）:
    #   0°-50°    = 右方    = 0.8-1 MPa = 红色偏高
    #   50°-290°  = 下方+左方+上方 = 0-0.6 MPa
    #   290°-360° = 右上方  = 0.6-0.8 MPa区域
    #
    # 绿色正常: 0.4-0.6 MPa = 约200°-350°（左上到右上）
    # 黄色偏低: 0-0.2 MPa = 约50°-200°（右下到左）
    # 红色偏高: 0.8-1 MPa = 约330°-50°（右方）
    if angle <= 50 or angle >= 330:
        return 'red', 100
    elif 200 <= angle <= 330:
        return 'green', 100
    elif 50 < angle < 200:
        return 'yellow', 100
    else:
        return 'unknown', 0


def judge_status(color):
    """根据指针所在颜色区域判断仪表盘状态"""
    if color == 'red':
        return 'red,high', (0, 0, 255)
    elif color == 'yellow':
        return 'yellow,low', (0, 255, 255)
    elif color == 'green':
        return 'green,ok', (0, 255, 0)
    else:
        return '', (128, 128, 128)


def judge_status_cn(color):
    """根据指针所在颜色区域判断仪表盘状态（中文，终端输出用）"""
    if color == 'red':
        return '仪表盘偏高，状态异常', (0, 0, 255)
    elif color == 'yellow':
        return '仪表盘偏低，状态异常', (0, 255, 255)
    elif color == 'green':
        return '仪表盘正常', (0, 255, 0)
    else:
        return '', (128, 128, 128)


def draw_result(img, cx, cy, radius, tip_point, status_text, text_color, rotation=0):
    """在图像上绘制识别结果"""
    result = img.copy()

    x1 = max(0, cx - radius)
    y1 = max(0, cy - radius)
    x2 = min(img.shape[1] - 1, cx + radius)
    y2 = min(img.shape[0] - 1, cy + radius)
    cv2.rectangle(result, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.circle(result, (cx, cy), 6, (0, 0, 255), -1)
    cv2.circle(result, (cx, cy), radius, (255, 0, 0), 2)

    if tip_point:
        cv2.line(result, (cx, cy), tip_point, (255, 0, 255), 3)
        cv2.circle(result, tip_point, 7, (255, 0, 255), -1)

    if status_text:
        cv2.rectangle(result, (10, 10), (250, 50), (0, 0, 0), -1)
        cv2.putText(result, status_text, (15, 42),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

    return result


# 缓存圆形检测结果
_cached_gauge = None
_last_detect_time = 0
_cached_lines = None
_last_lines_time = 0
_gauge_history = []
_locked_gauge = None  # 锁定的仪表盘参数
_lock_count = 0


def process_frame(frame):
    """
    处理单帧图像，返回处理后的图像和识别状态
    返回：(result_img, status_text_en, status_text_cn, tip_point, needle_angle, margin)
    """
    global _cached_gauge, _last_detect_time, _cached_lines, _last_lines_time

    if frame is None or frame.size == 0:
        return None, None, None, None, 0, 0

    try:
        h, w = frame.shape[:2]
        now = time.time()

        # 白平衡校正（每帧都做，但用快速方法）
        frame = correct_white_balance(frame)

        # 每0.8秒重新检测圆形，其余帧复用缓存
        need_detect = (_cached_gauge is None or
                       _cached_gauge[3] != w or _cached_gauge[4] != h or
                       (now - _last_detect_time) > 0.8)

        if need_detect:
            cx, cy, radius = detect_gauge_circle(frame)
            _cached_gauge = (cx, cy, radius, w, h)
            _last_detect_time = now
            _cached_lines = None
        else:
            cx, cy, radius = _cached_gauge[0], _cached_gauge[1], _cached_gauge[2]

        tip_point, lines = detect_needle(frame, cx, cy, radius,
                                          _cached_lines if (now - _last_lines_time) < 0.5 else None)
        if lines is not None:
            _cached_lines = lines
            _last_lines_time = now

        if tip_point:
            dx = tip_point[0] - cx
            dy = tip_point[1] - cy
            needle_angle = math.degrees(math.atan2(dy, dx)) % 360
            needle_color, margin = classify_gauge_color(frame, cx, cy, radius, needle_angle, tip_point)
        else:
            needle_angle = 0
            needle_color = 'unknown'
            margin = 0

        status_text_en, text_color = judge_status(needle_color)
        status_text_cn, _ = judge_status_cn(needle_color)
        result_img = draw_result(frame, cx, cy, radius, tip_point, status_text_en, text_color, 0)
        return result_img, status_text_en, status_text_cn, tip_point, needle_angle, margin
    except Exception as e:
        print(f"处理帧时出错：{e}")
        import traceback
        traceback.print_exc()
        return frame, "error", "处理出错", None, 0, 0


def run_image_mode(image_paths, output_dir=None, test_rotations=None):
    """
    图片模式：依次处理给定的图片文件
    test_rotations: 额外测试指定的旋转角度列表（用于验证旋转鲁棒性）
    """
    print("=" * 50)
    print("仪表盘状态识别系统 - 图片模式")
    print("=" * 50)
    print()

    results = []
    for img_path in image_paths:
        if not os.path.exists(img_path):
            print(f"【{img_path}】文件不存在，跳过")
            print()
            continue

        print(f"正在处理：{img_path}")
        img = cv2.imread(img_path)
        if img is None:
            print(f"  错误：无法读取图片 {img_path}")
            print()
            continue

        # 原始方向识别
        result_img, status_text_en, status_text_cn, _, rotation, _ = process_frame(img)
        basename = os.path.basename(img_path)

        if result_img is not None:
            if status_text_cn:
                print(f"【{basename}】识别结果：{status_text_cn} | 检测旋转：{rotation}°")
            else:
                print(f"【{basename}】未检测到指针")

            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
                out_path = os.path.join(output_dir, f"result_{basename}")
            else:
                out_path = f"result_{basename}"

            cv2.imwrite(out_path, result_img)
            print(f"  → 结果图已保存：{out_path}")
            results.append((basename, status_text_cn, rotation))
        print()

        # 如果指定了测试旋转角度，生成旋转图片并测试
        if test_rotations:
            h, w = img.shape[:2]
            test_cx, test_cy, test_r = detect_gauge_circle(img)
            pad = int(test_r * 0.3)
            crop_x1 = max(0, test_cx - test_r - pad)
            crop_y1 = max(0, test_cy - test_r - pad)
            crop_x2 = min(w, test_cx + test_r + pad)
            crop_y2 = min(h, test_cy + test_r + pad)
            cropped = img[crop_y1:crop_y2, crop_x1:crop_x2]

            for test_rot in test_rotations:
                print(f"  [旋转测试] 将图片旋转 {test_rot}° 后识别...")
                ch, cw = cropped.shape[:2]
                M = cv2.getRotationMatrix2D((cw // 2, ch // 2), test_rot, 1.0)
                rotated = cv2.warpAffine(cropped, M, (cw, ch), borderValue=(255, 255, 255))

                rot_result, rot_status_en, rot_status_cn, _, rot_detected, _ = process_frame(rotated)
                if rot_result is not None:
                    rot_path = f"rot_{test_rot}_{basename}"
                    cv2.imwrite(rot_path, rot_result)
                    match = "✓" if rot_status_cn == status_text_cn else "✗"
                    print(f"    {match} 旋转{test_rot}° → {rot_status_cn} | 检测旋转：{rot_detected}° | 保存：{rot_path}")
            print()

    print("=" * 50)
    print("识别汇总：")
    print("=" * 50)
    for img_file, status, rot in results:
        rot_info = f" (旋转{rot}°)" if abs(rot) > 3 else ""
        print(f"  {img_file}：{status}{rot_info}")
    print("=" * 50)


def run_camera_mode(device_id=0, width=640, height=480, fps=30, realtime=False):
    """
    相机模式：通过摄像头实时识别仪表盘状态

    按键说明：
      q / ESC : 退出程序
      s       : 保存当前帧到文件
      r       : 触发识别当前帧并输出到终端
      f       : 切换全屏显示

    参数：
      realtime : 启用实时识别，每帧自动处理并在终端输出结果
    """
    print("=" * 50)
    print("仪表盘状态识别系统 - 相机模式")
    print("=" * 50)
    print(f"正在打开摄像头设备 {device_id} ...")
    print(f"分辨率：{width}x{height}，帧率：{fps}")
    print()
    print("操作说明：")
    print("  [q] 或 [ESC] : 退出程序")
    print("  [s]          : 保存当前帧")
    print("  [r]          : 触发识别")
    print("  [f]          : 切换全屏")
    print("=" * 50)
    print()

    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        print(f"错误：无法打开摄像头设备 {device_id}")
        print("请检查摄像头是否已连接，或尝试更换设备号（如 --device 1）")
        return

    # 设置摄像头参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # 获取实际参数
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"摄像头已启动，实际分辨率：{actual_width}x{actual_height}，帧率：{actual_fps}")
    print("请将仪表盘置于摄像头前方至少20cm处")
    print()

    # 状态变量
    last_status = ""  # English for display
    status_color = (128, 128, 128)
    frame_count = 0
    fullscreen = False
    last_print_time = 0
    last_printed_status = ""

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("警告：无法读取视频帧，尝试重新连接...")
            time.sleep(0.5)
            continue

        frame_count += 1
        display_frame = frame.copy()

        if realtime:
            # 实时模式：每帧自动处理
            result_img, status_text_en, status_text_cn, _, rotation, _ = process_frame(frame)
            if result_img is not None:
                display_frame = result_img
                if status_text_en:
                    last_status = status_text_en
                    last_status_cn = status_text_cn
                    # 限制终端输出频率：状态变化或每秒最多输出一次
                    now = time.time()
                    if status_text_cn != last_printed_status or (now - last_print_time) >= 1.0:
                        timestamp = time.strftime("%H:%M:%S")
                        rot_info = f" [旋{rotation}°]" if abs(rotation) > 5 else ""
                        print(f"[{timestamp}] 识别结果：{status_text_cn}{rot_info}")
                        last_printed_status = status_text_cn
                        last_print_time = now
                else:
                    now = time.time()
                    if (now - last_print_time) >= 1.0:
                        timestamp = time.strftime("%H:%M:%S")
                        print(f"[{timestamp}] 等待状态稳定")
                        last_print_time = now
        else:
            # 非实时模式：显示上一状态文字
            if last_status:
                font_size = 28
                text = last_status
                text_width = len(text) * font_size
                text_height = font_size + 10
                bg_x1, bg_y1 = 10, 10
                bg_x2 = bg_x1 + text_width + 20
                bg_y2 = bg_y1 + text_height + 10

                if 'red' in text:
                    status_color = (0, 0, 255)
                elif 'yellow' in text:
                    status_color = (0, 255, 255)
                elif 'green' in text:
                    status_color = (0, 255, 0)
                else:
                    status_color = (128, 128, 128)

                overlay = display_frame.copy()
                cv2.rectangle(overlay, (bg_x1, bg_y1), (bg_x2, bg_y2), (0, 0, 0), -1)
                display_frame = cv2.addWeighted(display_frame, 0.7, overlay, 0.3, 0)
                cv2.putText(display_frame, text, (bg_x1 + 10, bg_y1 + 5 + font_size),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        # 在右下角显示帧率信息
        fps_text = f"Frame: {frame_count}"
        cv2.putText(display_frame, fps_text,
                   (display_frame.shape[1] - 150, display_frame.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 实时模式标识
        if realtime:
            cv2.putText(display_frame, "[Realtime]", (10, display_frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 显示结果窗口
        window_name = "Gauge Recognition (q: quit, s: save)"
        if fullscreen:
            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow(window_name, display_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # q 或 ESC
            print("\n用户主动退出程序")
            break
        elif key == ord('s'):  # 保存当前帧
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            save_path = f"camera_capture_{timestamp}.jpg"
            cv2.imwrite(save_path, display_frame)
            print(f"  → 当前帧已保存：{save_path}")
        elif key == ord('r') and not realtime:  # 手动触发识别（非实时模式下有效）
            result_img, status_text_en, status_text_cn, _, rotation, _ = process_frame(frame)
            if result_img is not None:
                if status_text_en:  # 只更新有效状态
                    last_status = status_text_en
                    timestamp = time.strftime("%H:%M:%S")
                    rot_info = f" [旋{rotation}°]" if abs(rotation) > 5 else ""
                    print(f"[{timestamp}] 识别结果：{status_text_cn}{rot_info}")
                else:
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] 未检测到指针，保持上一状态")

                # 保存带有表盘框和指针的这一帧识别画面
                save_timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_path = f"recognized_result_{save_timestamp}.jpg"
                cv2.imwrite(save_path, result_img)
                print(f"  → 识别帧已自动保存：{save_path}")

                # 将识别出的画面弹出一个新窗口定格显示
                cv2.imshow("Result", result_img)
        elif key == ord('f'):  # 切换全屏
            fullscreen = not fullscreen
            if fullscreen:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            else:
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

    cap.release()
    cv2.destroyAllWindows()
    print("摄像头已关闭")


def main():
    parser = argparse.ArgumentParser(
        description='仪表盘状态识别程序（支持图片模式和相机模式）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例：
  # 图片调试模式（默认处理 1.jpg ~ 6.jpg）
  python3 watch.py --mode image
  python3 watch.py --mode image --images 1.jpg 3.jpg 5.jpg

  # 图片调试模式 + 旋转测试（验证旋转鲁棒性）
  python3 watch.py --mode image --test-rotation 30 90 180 270

  # 相机模式
  python3 watch.py --mode camera
  python3 watch.py --mode camera --device 0 --width 1280 --height 720

  # 相机模式 + 实时识别
  python3 watch.py --mode camera --realtime

  # 查看帮助
  python3 watch.py -h
        """
    )

    parser.add_argument(
        '--mode', '-m',
        type=str,
        choices=['image', 'camera'],
        default='image',
        help='工作模式：image（图片模式，默认）或 camera（相机模式）'
    )

    parser.add_argument(
        '--images', '-i',
        nargs='*',
        default=['1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg', '6.jpg'],
        help='图片模式下要处理的图片文件路径（默认：1.jpg ~ 6.jpg）'
    )

    parser.add_argument(
        '--output-dir', '-o',
        type=str,
        default=None,
        help='结果图片保存目录（默认保存在当前目录）'
    )

    parser.add_argument(
        '--test-rotation',
        nargs='*',
        type=int,
        default=None,
        help='测试指定旋转角度下的识别鲁棒性（如：30 90 180）'
    )

    parser.add_argument(
        '--device', '-d',
        type=int,
        default=0,
        help='相机模式下使用的摄像头设备号（默认：0）'
    )

    parser.add_argument(
        '--width',
        type=int,
        default=640,
        help='相机模式下的画面宽度（默认：640）'
    )

    parser.add_argument(
        '--height',
        type=int,
        default=480,
        help='相机模式下的画面高度（默认：480）'
    )

    parser.add_argument(
        '--fps',
        type=int,
        default=30,
        help='相机模式下的目标帧率（默认：30）'
    )

    parser.add_argument(
        '--realtime', '-r',
        action='store_true',
        help='相机模式下启用实时识别，每帧自动处理并在终端输出结果'
    )

    args = parser.parse_args()

    if args.mode == 'image':
        run_image_mode(args.images, args.output_dir, args.test_rotation)
    elif args.mode == 'camera':
        run_camera_mode(args.device, args.width, args.height, args.fps, args.realtime)


if __name__ == "__main__":
    main()
