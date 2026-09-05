import os
import sys
import threading
from unittest.mock import MagicMock
from PIL import ImageFont
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
# 3. 正常导入 xgoedu 库并初始化
from xgoedu import XGOEDU
XGO_edu = XGOEDU()
# =========================================================
# 4. 多线程后台语音播报函数
def text_to_speech_async(text):
    def speech_worker():
        print("【后台线程】开始语音合成播报:", text)
        XGO_edu.SpeechSynthesis(text)
        print("【后台线程】语音播放完毕！")
    thread = threading.Thread(target=speech_worker, daemon=True)
    thread.start()
    return thread

# =========================================================
if __name__ == "__main__":
    # 后台异步播放
    t = text_to_speech_async("三人佩戴安全帽，两人没有佩戴安全帽")
    t.join()  # 等待播报结束