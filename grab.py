from xgolib import XGO
from xgoedu import XGOEDU
import time

# 初始化机器狗
dog = XGO("xgolite")
XGO_edu = XGOEDU()

# --- 基础配置参数 ---
READY_POS = [50, 50]    # 手臂安全收回/待机位置 [X, Z]
LIFT_HEIGHT = 70        # 抬起/安全过渡高度 Z (mm)

# 爪子开合度 (根据实际物体大小微调，避免舵机堵转)
CLAW_OPEN = 0           # 爪子打开 (0~255)
CLAW_CLOSE = 250        # 爪子抓紧 (建议 200-240，防止死锁发热)

def arm_grab(target_x, target_z, lift_z=LIFT_HEIGHT):
    """
    优化后的抓取动作 (门字型轨迹)
    :param target_x: 抓取点 X 坐标 (前伸距离 0~150mm)
    :param target_z: 抓取点 Z 坐标 (高度 -20~150mm)
    :param lift_z: 抬起安全高度
    """
    print(f"--> [抓取] 目标位置: X={target_x}, Z={target_z}")
    
    # 1. 预备：张开爪子，手臂收回预备位
    dog.claw(CLAW_OPEN)
    dog.arm(READY_POS[0], READY_POS[1])
    time.sleep(0.8)
    
    # 2. 移动到目标点正上方 (避免斜向撞到物体)
    dog.arm(target_x, lift_z)
    time.sleep(1.0)
    
    # 3. 垂直下降到抓取点
    dog.arm(target_x, target_z)
    time.sleep(1.0)
    
    # 4. 闭合爪子抓紧物体
    dog.claw(CLAW_CLOSE)
    time.sleep(1.8)
    
    # 5. 垂直抬起物体
    dog.arm(target_x, lift_z)
    time.sleep(1.0)
    
    # 6. 收回手臂至重心安全位置（带物行走/准备转身）
    dog.arm(READY_POS[0], READY_POS[1])
    time.sleep(1.0)

def arm_place(target_x, target_z, lift_z=LIFT_HEIGHT):
    """
    优化后的放置动作 (门字型轨迹)
    :param target_x: 放置点 X 坐标
    :param target_z: 放置点 Z 坐标
    :param lift_z: 抬起安全高度
    """
    print(f"--> [放置] 目标位置: X={target_x}, Z={target_z}")
    
    # 1. 带着物体移动到放置点正上方
    dog.arm(target_x, lift_z)
    time.sleep(1.0)
    
    # 2. 垂直下降到放置点
    dog.arm(target_x, target_z)
    time.sleep(1.0)
    
    # 3. 张开爪子释放物体
    dog.claw(CLAW_OPEN)
    time.sleep(1.0)
    
    # 4. 垂直抬起空手臂 (避免刮倒刚放下的物体)
    dog.arm(target_x, lift_z)
    time.sleep(1.0)
    
    # 5. 复位收回手臂
    dog.arm(READY_POS[0], READY_POS[1])
    time.sleep(1.0)

# --- 主程序逻辑 ---
if __name__ == "__main__":
    try:
        # 1. 初始化站立姿态
        print("初始化站立...")
        dog.action(255)  # 复位站立
        time.sleep(1.5)
        
        # 2. 执行抓取 (指定抓取点: X=120, Z=-10)
        arm_grab(target_x=120, target_z=-10)
        time.sleep(0.5)
        
        """
        # 3. 转身搬运（转向 45 度）
        print("转向放置区...")
        dog.turn_by(45, 1.5)  # 旋转 45 度，用时 1.5 秒
        time.sleep(1.5)
        """
        
        # 4. 执行放置 (指定放置点: X=110, Z=0)
        arm_place(target_x=110, target_z=0)
        time.sleep(0.5)
        
        # 0. 初始化站立姿态
        print("任务完成回初始化")
        dog.action(255)  # 复位站立

    except KeyboardInterrupt:
        print("程序被手动中断")
    finally:
        # 安全退出：复位并关闭总线
        dog.arm(READY_POS[0], READY_POS[1])
        dog.claw(CLAW_OPEN)
        dog.stop()