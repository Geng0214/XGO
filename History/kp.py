from xgolib import XGO
import time

# 初始化
dog = XGO("xgomini")

yaw = dog.read_yaw()





def walk_straight(speed, duration):

    # 锁定初始偏航角
    time.sleep(0.5)  # 等待陀螺仪稳定
    target_yaw = 0
    for _ in range(5):
        target_yaw = dog.read_yaw()
        time.sleep(0.01)
    print(f">>> 航向锁定成功: {target_yaw}")
    
    # PID参数
    kp = 1.2 
    
    start_time = time.time()
    
    try:
        while time.time() - start_time < duration:
            # 读取当前角度
            current_yaw = dog.read_yaw()
            # 计算偏差
            error = target_yaw - current_yaw
            
            # 计算旋转修正(纠偏)
            yaw_correction = int(error * kp)
            
            # 限制修正范围，防止转得太快失去平衡
            if yaw_correction > 40: yaw_correction = 40
            if yaw_correction < -40: yaw_correction = -40
            
            # 同时调用两个函数：一个控前进，一个控旋转
            dog.move('x', speed)   # 控制前后
            dog.turn(yaw_correction) # 控制纠偏
            
            # 监控输出
            print(f"Yaw: {current_yaw:.1f} | Error: {error:.1f} | Corr: {yaw_correction}", end='\r')
            
            time.sleep(0.04) # 25Hz控制频率
    except Exception as e:
        print(f"\n运行中出错: {e}")
    finally:
        # 停止运动
        dog.stop()
        print("\n运行结束。")


# ================= 主程序 =================
try:

    print("开始直线前进...")
    walk_straight(speed=30, duration=10)   
    time.sleep(3)
    
    dog.turn(180)
    time.sleep(3)

    dog.move('y', 25)
    time.sleep(2)

    # print("开始直线后退...")
    # walk_straight(speed=-30, duration=10)

except Exception as e:
    print(f"\n报错信息: {e}")
finally:
    dog.stop()
