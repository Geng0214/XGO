"""识别器模块 - 从 cam_server 分离出来的独立识别逻辑"""

from .gauge import GaugeRecognizer
from .digit import DigitRecognizer
from .helmet import HelmetRecognizer
from .letter import LetterRecognizer

RECOGNIZERS = {
    "gauge": GaugeRecognizer,
    "digit": DigitRecognizer,
    "helmet": HelmetRecognizer,
    "letter": LetterRecognizer,
}

__all__ = ["RECOGNIZERS", "GaugeRecognizer", "DigitRecognizer", "HelmetRecognizer", "LetterRecognizer"]
