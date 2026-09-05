"""区域字母识别 - 模板匹配"""

import cv2
import numpy as np
import os
from config import LETTER_MATCH_THRESHOLD


class LetterDetector:
    def __init__(self, template_dir="templates/"):
        self.templates = {}
        self.template_dir = template_dir
        for letter in ["A", "B", "C", "D"]:
            path = os.path.join(template_dir, f"{letter}.png")
            if os.path.exists(path):
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    self.templates[letter] = img
        print(f"Loaded {len(self.templates)} letter templates: {list(self.templates.keys())}")

    def detect(self, frame):
        """识别区域字母，返回 'A'/'B'/'C'/'D' 或 None"""
        if not self.templates:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        best_letter = None
        best_score = -1

        h, w = gray.shape[:2]
        roi = gray[h // 4:3 * h // 4, w // 4:3 * w // 4]

        for letter, tmpl in self.templates.items():
            th, tw = tmpl.shape[:2]
            if th > roi.shape[0] or tw > roi.shape[1]:
                tmpl_resized = cv2.resize(tmpl, (min(tw, roi.shape[1]), min(th, roi.shape[0])))
            else:
                tmpl_resized = tmpl

            result = cv2.matchTemplate(roi, tmpl_resized, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(result)

            if max_val > best_score:
                best_score = max_val
                best_letter = letter

        if best_score >= LETTER_MATCH_THRESHOLD:
            return best_letter
        return None
