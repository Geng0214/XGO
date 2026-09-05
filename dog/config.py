"""全局参数配置"""

# ===================== 摄像头 =====================
CAMERA_INDEX = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# ===================== 巡线参数 =====================
LINE_SPEED = 12
PID_KP = 50.0
PID_KI = 0.0
PID_KD = 30.0
BLACK_LINE_HSV_LOW = (0, 0, 0)
BLACK_LINE_HSV_HIGH = (180, 255, 60)

# ===================== 识别参数 =====================
SCAN_CONFIRM_FRAMES = 10
LETTER_MATCH_THRESHOLD = 0.5

# ===================== 仪表盘颜色范围 (HSV) =====================
GAUGE_RED_LOW = (0, 120, 70)
GAUGE_RED_HIGH = (10, 255, 255)
GAUGE_YELLOW_LOW = (15, 100, 70)
GAUGE_YELLOW_HIGH = (35, 255, 255)
GAUGE_GREEN_LOW = (35, 100, 70)
GAUGE_GREEN_HIGH = (85, 255, 255)

# 数显表计阈值
DIGIT_LOW_THRESH = 20000
DIGIT_HIGH_THRESH = 30000

# ===================== 安全帽检测 =====================
HELMET_CONF = 0.85
HELMET_TARGET_COUNT = 5

# ===================== 语音播报 =====================
TTS_VOICE = "zh-CN-XiaoxiaoNeural"
TTS_CACHE_DIR = "sounds"

# ===================== 通信参数 =====================
UDP_PORT = 6001
UDP_BROADCAST_IP = "255.255.255.255"

# ===================== 巡检目标 =====================
INSPECTION_TARGETS = ["A", "B", "C", "D"]
