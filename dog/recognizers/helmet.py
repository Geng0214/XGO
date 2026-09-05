"""安全帽识别器"""


class HelmetRecognizer:
    def __init__(self):
        from vision.helmet_detect import HelmetDetector, DEFAULT_MODEL
        self.detector = HelmetDetector(DEFAULT_MODEL)

    def process(self, frame):
        # process_frame 内部完成推理、跟踪、画框与统计，返回画好框的帧
        return self.detector.process_frame(frame)
