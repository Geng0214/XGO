"""字母识别器 - 用于 cam_server"""

import cv2


class LetterRecognizer:
    def __init__(self):
        from vision.letter_detect import LetterDetector
        self.detector = LetterDetector()

    def process(self, frame):
        """识别并叠加结果到画面"""
        display = frame.copy()
        letter = self.detector.detect(frame)

        if letter:
            cv2.putText(display, f"Letter: {letter}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Letter: None",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return display
