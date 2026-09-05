"""数显表计OCR识别

机械式数显表，7位数字，白字黑底。
策略：多阈值提取连通域 → 去重+合并 → 按y聚类找数字行 → 模板匹配识别
"""

import cv2
import numpy as np
from config import DIGIT_LOW_THRESH, DIGIT_HIGH_THRESH

# 预置模板：从测试图片中提取的标准化数字模板
# 每个数字取多个样本的平均值
_TEMPLATES = None


def _build_templates():
    """从已知图片构建模板库"""
    import os
    template_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "digit")
    known = [
        ("0035212.png", "0035212"),
        ("0023281.png", "0023281"),
        ("0010561.png", "0010561"),
    ]
    templates = {}  # digit -> [img20x36, ...]

    for fname, value in known:
        path = os.path.join(template_dir, fname)
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        digits = _extract_digits_from_image(gray)
        if digits is None or len(digits) != 7:
            continue
        for i, dimg in enumerate(digits):
            d = int(value[i])
            if d not in templates:
                templates[d] = []
            templates[d].append(dimg)

    # 保留所有样本（不用平均，避免异常样本污染）
    return templates


def _get_templates():
    global _TEMPLATES
    if _TEMPLATES is None:
        _TEMPLATES = _build_templates()
    return _TEMPLATES


class DigitDetector:
    def __init__(self):
        pass

    def detect(self, frame):
        """识别数显表计读数，返回 {"value": int, "value_str": str, "status": str} 或 None"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        digits = _extract_digits_from_image(gray)
        if digits is None or len(digits) != 7:
            return None

        templates = _get_templates()
        if not templates:
            return None

        value_str = ""
        for dimg in digits:
            d = _match_template(dimg, templates)
            value_str += str(d)

        if len(value_str) != 7:
            return None

        try:
            value = int(value_str)
        except ValueError:
            return None

        if value < DIGIT_LOW_THRESH:
            status = "偏低"
        elif value > DIGIT_HIGH_THRESH:
            status = "偏高"
        else:
            status = "正常"

        return {"value": value, "value_str": value_str, "status": status}


def _find_all_candidates(gray):
    """多阈值找所有数字候选"""
    img_h, img_w = gray.shape
    all_cands = []
    for thresh in [60, 80, 100, 120]:
        _, binary = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
        num_labels, _, stats, _ = cv2.connectedComponentsWithStats(binary)
        for i in range(1, num_labels):
            x, y, cw, ch, area = stats[i]
            if cw > img_w * 0.8 or ch > img_h * 0.8:
                continue
            if area < 50:
                continue
            ratio = ch / max(cw, 1)
            if ratio < 0.5 or ratio > 10:
                continue
            all_cands.append((x, y, cw, ch, area))
    return all_cands


def _dedup_and_merge(candidates):
    """去重+合并重叠组件（同一数字被多阈值检测到多个版本）"""
    if not candidates:
        return []

    # 按面积从大到小排
    candidates.sort(key=lambda d: -d[4])

    result = []
    used = [False] * len(candidates)

    for i in range(len(candidates)):
        if used[i]:
            continue
        x, y, w, h, area = candidates[i]
        for j in range(i + 1, len(candidates)):
            if used[j]:
                continue
            x2, y2, w2, h2, a2 = candidates[j]
            # 计算重叠
            ox = max(0, min(x + w, x2 + w2) - max(x, x2))
            oy = max(0, min(y + h, y2 + h2) - max(y, y2))
            overlap_area = ox * oy
            min_area = min(w * h, w2 * h2)
            # 只有当重叠面积占较小组件50%以上才合并（同一数字）
            if min_area > 0 and overlap_area / min_area > 0.5:
                nx = min(x, x2)
                ny = min(y, y2)
                nw = max(x + w, x2 + w2) - nx
                nh = max(y + h, y2 + h2) - ny
                x, y, w, h = nx, ny, nw, nh
                used[j] = True
        result.append((x, y, w, h))

    return result


def _find_digit_row(candidates):
    """按y聚类，找数字行（优先选高度一致的行）"""
    if len(candidates) < 7:
        return None

    sorted_cands = sorted(candidates, key=lambda d: d[1])

    # 聚类：y差值<20px归为同一行
    rows = []
    current = [sorted_cands[0]]
    for c in sorted_cands[1:]:
        if abs(c[1] - current[-1][1]) < 20:
            current.append(c)
        else:
            rows.append(current)
            current = [c]
    rows.append(current)

    # 评分：优先选有7+个高度一致项的行
    best_row, best_score = None, -999
    for row in rows:
        n = len(row)
        if n < 5:
            continue
        heights = [d[3] for d in row]
        median_h = np.median(heights)
        # 高度一致的项数（在中位数0.5x-2x范围内）
        consistent = sum(1 for h in heights if 0.5 * median_h < h < 2.0 * median_h)
        # 必须有>=7个一致项才考虑
        if consistent < 7:
            continue
        # 评分：一致项数越多越好，越接近7越好
        score = consistent * 10 - abs(n - 7) * 5
        if score > best_score:
            best_score = score
            best_row = row

    return best_row


def _filter_row_by_height(row):
    """过滤行内高度不一致的噪声"""
    if len(row) <= 7:
        return row
    heights = [d[3] for d in row]
    median_h = np.median(heights)
    # 只保留高度在中位数0.5x-2x范围内的
    filtered = [d for d in row if 0.5 * median_h < d[3] < 2.0 * median_h]
    return filtered if len(filtered) >= 7 else row


def _find_display_window(gray):
    """定位暗色的数显表显示窗口，返回 (x1,y1,x2,y2) 或 None"""
    h, w = gray.shape
    # 只对大图做窗口检测（小图直接认为整张就是显示区）
    if w < 600:
        return None

    inv = 255 - gray
    _, binary = cv2.threshold(inv, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 闭合填充
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 10))
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    img_area = h * w
    best = None
    best_score = 0
    for cnt in contours:
        x, y, cw, ch = cv2.boundingRect(cnt)
        area = cw * ch
        ratio = cw / max(ch, 1)
        # 显示窗口特征：宽>高，面积占5%-60%
        if ratio < 1.2 or area < img_area * 0.03 or area > img_area * 0.7:
            continue
        score = area
        if score > best_score:
            best_score = score
            best = (x, y, x + cw, y + ch)

    return best


def _extract_digits_from_image(gray):
    """从灰度图中提取7个标准化数字图像"""
    # 大图先定位显示窗口
    window = _find_display_window(gray)
    if window is not None:
        x1, y1, x2, y2 = window
        gray = gray[y1:y2, x1:x2]

    # 方法1：投影分割（适合窗口裁剪后的小图）
    digits = _extract_by_projection(gray)
    if digits is not None and len(digits) == 7:
        return digits

    # 方法2：连通域法（回退）
    return _extract_by_components(gray)


def _extract_by_projection(gray):
    """在显示窗口内用连通域+尺寸过滤提取7个数字"""
    h, w = gray.shape
    # 阈值110提取亮区
    _, binary = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY)

    # 连通域
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)

    # 收集数字候选
    candidates = []
    for i in range(1, num_labels):
        x, y, cw, ch, area = stats[i]
        # 数字尺寸过滤：宽>3, 高>8, 面积>20
        if cw < 3 or ch < 8 or area < 20:
            continue
        # 宽高比：不能太扁
        ratio = ch / max(cw, 1)
        if ratio < 0.5 or ratio > 8:
            continue
        candidates.append((x, y, cw, ch))

    if len(candidates) < 7:
        return None

    # 按x排序
    candidates.sort(key=lambda d: d[0])

    # 取7个最合理的
    if len(candidates) > 7:
        # 去掉太宽的框架（宽度>中位数3倍）
        widths = [c[2] for c in candidates]
        median_w = np.median(widths)
        filtered = [c for c in candidates if c[2] < 3.0 * median_w]
        if len(filtered) >= 7:
            candidates = filtered
        # 取最左边7个
        candidates = candidates[:7]

    if len(candidates) != 7:
        return None

    # 提取每个数字
    digits = []
    for x, y, cw, ch in candidates:
        pad_x = max(1, int(cw * 0.15))
        pad_y = max(1, int(ch * 0.1))
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(w, x + cw + pad_x)
        y2 = min(h, y + ch + pad_y)
        region = gray[y1:y2, x1:x2]
        _, digit_bin = cv2.threshold(region, 110, 255, cv2.THRESH_BINARY)

        # 内容居中
        rows_mask = np.any(digit_bin > 0, axis=1)
        cols_mask = np.any(digit_bin > 0, axis=0)
        if np.any(rows_mask) and np.any(cols_mask):
            r_min, r_max = np.where(rows_mask)[0][[0, -1]]
            c_min, c_max = np.where(cols_mask)[0][[0, -1]]
            digit_crop = digit_bin[r_min:r_max+1, c_min:c_max+1]
        else:
            digit_crop = digit_bin

        canvas = np.zeros((36, 20), dtype=np.uint8)
        dh, dw = digit_crop.shape
        if dh > 0 and dw > 0:
            scale = min(18.0 / dw, 32.0 / dh)
            new_w = max(1, int(dw * scale))
            new_h = max(1, int(dh * scale))
            resized = cv2.resize(digit_crop, (new_w, new_h))
            ox = (20 - new_w) // 2
            oy = (36 - new_h) // 2
            canvas[oy:oy+new_h, ox:ox+new_w] = resized
        digits.append(canvas)

    return digits


def _extract_by_components(gray):
    """连通域法提取数字（回退方案）"""
    cands = _find_all_candidates(gray)
    if len(cands) < 7:
        return None

    merged = _dedup_and_merge(cands)
    row = _find_digit_row(merged)
    if row is None:
        return None

    row = _filter_row_by_height(row)
    row.sort(key=lambda d: d[0])

    if len(row) > 7:
        mid_x = np.mean([d[0] + d[2] / 2 for d in row])
        row.sort(key=lambda d: abs(d[0] + d[2] / 2 - mid_x))
        row = row[:7]
        row.sort(key=lambda d: d[0])

    if len(row) != 7:
        return None

    h, w = gray.shape
    digits = []
    for x, y, cw, ch in row:
        pad_x = max(1, int(cw * 0.15))
        pad_y = max(1, int(ch * 0.1))
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(w, x + cw + pad_x)
        y2 = min(h, y + ch + pad_y)
        region = gray[y1:y2, x1:x2]
        _, binary = cv2.threshold(region, 80, 255, cv2.THRESH_BINARY)

        rows_mask = np.any(binary > 0, axis=1)
        cols_mask = np.any(binary > 0, axis=0)
        if np.any(rows_mask) and np.any(cols_mask):
            r_min, r_max = np.where(rows_mask)[0][[0, -1]]
            c_min, c_max = np.where(cols_mask)[0][[0, -1]]
            digit_crop = binary[r_min:r_max+1, c_min:c_max+1]
        else:
            digit_crop = binary

        canvas = np.zeros((36, 20), dtype=np.uint8)
        dh, dw = digit_crop.shape
        if dh > 0 and dw > 0:
            scale = min(18.0 / dw, 32.0 / dh)
            new_w = max(1, int(dw * scale))
            new_h = max(1, int(dh * scale))
            resized_crop = cv2.resize(digit_crop, (new_w, new_h))
            ox = (20 - new_w) // 2
            oy = (36 - new_h) // 2
            canvas[oy:oy+new_h, ox:ox+new_w] = resized_crop
        digits.append(canvas)

    return digits


def _match_template(digit_img, templates):
    """模板匹配，对每个数字的所有样本取最佳匹配，再选最高分的数字"""
    best_digit = 0
    best_score = -1
    a = digit_img.astype(np.float32).flatten()

    for d, samples in templates.items():
        for tmpl in samples:
            b = tmpl.astype(np.float32).flatten()
            # 直接像素差值
            score = 1.0 - np.mean(np.abs(a - b)) / 255.0
            if score > best_score:
                best_score = score
                best_digit = d
    return best_digit


if __name__ == "__main__":
    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    test_images = [
        "digit/0035212.png",
        "digit/0023281.png",
        "digit/0010561.png",
    ]

    det = DigitDetector()
    all_pass = True
    for path in test_images:
        img = cv2.imread(path)
        if img is None:
            print(f"无法读取: {path}")
            continue
        result = det.detect(img)
        basename = os.path.basename(path).split('.')[0]
        if result:
            match = result['value_str'] == basename
            status = "OK" if match else "FAIL"
            print(f"{basename} -> {result['value_str']} ({result['status']}) [{status}]")
            if not match:
                all_pass = False
        else:
            print(f"{basename} -> 识别失败")
            all_pass = False

    print(f"\n{'全部通过!' if all_pass else '有错误，需继续优化'}")
