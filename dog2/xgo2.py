from xgolib import XGO
import time
from xgoedu import XGOEDU

# 实例化 AI 教育库
edu = XGOEDU()
# 实例化 XGO 控制库
dog = XGO("xgomini")

def squat_standard():
    # 1. 动作复位
    print("正在复位...")
    dog.action(255)
    time.sleep(1)
    
    # 2. 将高度降至最低极限 75mm
    print("正在执行标准化低趴 (75mm)...")
    dog.translation('z', 75)
    time.sleep(1)


# --- 关键修正：使用 edu 对象调用摄像头 ---
try:
    print("正在打开摄像头...")
    # 注意：XGOEDU 的摄像头通常是把画面显示在小狗背部的 AI 模块屏幕上
    # 如果要保存文件，确保有写权限
    edu.cameraOn(filename="camera") 
    
    # 执行低趴动作
    squat_standard()
    
    # 维持一段时间观察效果
    time.sleep(5)

except Exception as e:
    print(f"出错啦: {e}")

finally:
    # 建议加上关闭摄像头的操作
    # edu.cameraOff() 
    dog.stop()