"""仪表盘识别器"""

import time


class GaugeRecognizer:
    def __init__(self):
        from watch import process_frame
        self.process_frame = process_frame
        self.last_status = ""
        self.frame_count = 0
        self.last_print_time = 0

    def process(self, frame):
        result_img, status_text_en, status_text_cn, tip, angle, margin = self.process_frame(frame)
        self.frame_count += 1

        now = time.time()
        if status_text_cn and (status_text_cn != self.last_status or (now - self.last_print_time) >= 2.0):
            print(f"[Frame {self.frame_count}] {status_text_cn} (angle={angle:.0f} margin={margin:.0f})")
            self.last_status = status_text_cn
            self.last_print_time = now

        return result_img if result_img is not None else frame
