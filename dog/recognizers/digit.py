"""数显表计识别器"""

import cv2


class DigitRecognizer:
    def __init__(self):
        from vision.digit_detect import DigitDetector
        self.detector = DigitDetector()

    def process(self, frame):
        result = self.detector.detect(frame)

        if result:
            value = result['value']
            status = result['status']

            # 终端输出
            if status == "偏低":
                print(f"数显表计显示{value}，状态异常（偏低）")
            elif status == "偏高":
                print(f"数显表计显示{value}，状态异常（偏高）")
            else:
                print(f"数显表计显示{value}，状态正常")

            # 画面显示
            if status == "正常":
                label = "OK"
                color = (0, 255, 0)  # 绿色
            elif status == "偏低":
                label = "LOW"
                color = (0, 0, 255)  # 红色
            else:
                label = "HIGH"
                color = (0, 165, 255)  # 橙色

            # 数值
            cv2.putText(frame, f"Value: {value}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # 状态标签（大字）
            cv2.putText(frame, label, (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

        else:
            cv2.putText(frame, "No digit", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return frame
