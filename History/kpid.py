from xgolib import XGO
import time

# ================= 调试参数 =================
KP = 1.2
KI = 0.05  
KD = 0.5   

ACCEL_TIME = 1.5  # 加速时间
DECEL_TIME = 1.5  # 减速时间

ANGLE_TOLERANCE = 1.5  # 角度死区：误差小于 1.5 度就不动了
MIN_TURN_OUTPUT = 8    # 最小输出阈值：PID 输出小于 8 时不发指令，防止碎步漂移

CONTROL_FREQ = 0.04  # 25Hz
# ============================================

dog = XGO("xgomini")
dog.pace('normal')

class IncrementalPID:
    def __init__(self, P, I, D):
        self.Kp, self.Ki, self.Kd = P, I, D
        self.PIDOutput = 0.0
        self.Error = 0.0  
        self.LastError = 0.0
        self.LastLastError = 0.0

    def Compute(self, target, current):
        error = target - current
        while error > 180: error -= 360
        while error < -180: error += 360
        self.Error = error 

        increment = (self.Kp * (self.Error - self.LastError) + 
                     self.Ki * error + 
                     self.Kd * (self.Error - 2 * self.LastError + self.LastLastError))
        
        self.PIDOutput += increment
        self.PIDOutput = max(min(self.PIDOutput, 50), -50)
        self.LastLastError = self.LastError
        self.LastError = self.Error
        return int(self.PIDOutput)

def move_straight(speed, duration, target_yaw):
    pid = IncrementalPID(KP, KI, KD)
    
    # 保护逻辑：防止总时间太短
    actual_accel = min(ACCEL_TIME, duration / 2)
    actual_decel = min(DECEL_TIME, duration / 2)
    
    start_time = time.time()
    print(f"\n[前进] 目标航向: {target_yaw:.1f}")

    try:
        while True:
            now = time.time()
            elapsed = now - start_time
            if elapsed >= duration:
                break
            
            if elapsed < actual_accel:
                current_speed = speed * (elapsed / actual_accel)
            elif elapsed > (duration - actual_decel):
                remaining_time = duration - elapsed
                current_speed = speed * (remaining_time / actual_decel)
            else:
                current_speed = speed

            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)

            dog.move('x', int(current_speed))   
            dog.turn(correction)   
            
            print(f"Speed:{int(current_speed):>3} | Cur:{current_yaw:>6.1f} | Err:{pid.Error:>5.1f} ", end='\r')
            time.sleep(CONTROL_FREQ)

    finally:
        dog.move('x', 0)
        dog.turn(0)
        time.sleep(0.3)
        dog.stop()

def turn_to_angle(target_yaw, timeout=8.0): 
    pid = IncrementalPID(KP, KI, KD)
    start_time = time.time()
    arrival_count = 0 
    
    print(f"\n[转向] 目标角度: {target_yaw:.1f}")
    
    try:
        while time.time() - start_time < timeout:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)
            
            # 优化逻辑 1: 误差死区 
            if abs(pid.Error) < ANGLE_TOLERANCE:
                # 已经进入误差允许范围，直接给 0 指令
                dog.move('x', 0)
                dog.turn(0)
                arrival_count += 1
            else:
                # 最小输出限制 
                if abs(correction) < MIN_TURN_OUTPUT:
                    actual_turn = 0
                else:
                    actual_turn = correction
                
                dog.move('x', 0)
                dog.turn(actual_turn)
                arrival_count = 0
            
            print(f"Turning.. Cur:{current_yaw:>6.1f} | Err:{pid.Error:>5.1f} | Out:{correction:>3}", end='\r')
            
            # 连续 8 次（约 0.3 秒）稳定在范围内则认为完成
            if arrival_count > 8: 
                break  
            
            time.sleep(CONTROL_FREQ)
            
    finally:
        # 强制停止并等待物理稳定
        dog.move('x', 0)
        dog.turn(0)
        dog.stop() 
        time.sleep(0.5) # 给机器人半秒时间完全站稳，消除惯性
        print(f"\n转向完成,最终角度: {dog.read_yaw():.1f}")

def move_sideways(speed_y, duration, target_yaw, x_compensation=3):
    """
    speed_y: 正数为向左
    duration: 持续时间
    target_yaw: 保持的角度
    x_compensation: X轴补偿值。如果狗侧移时后退给正数 如果前冲给负数。
    """
    pid = IncrementalPID(KP, KI, KD)
    start_time = time.time()
    
    # 打印当前的补偿设置，方便调试
    print(f"\n[横移] 侧移速度: {speed_y} | 航向锁定: {target_yaw:.1f} | X轴补偿: {x_compensation}")
    
    try:
        while time.time() - start_time < duration:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)
            
            dog.move('y', speed_y) 
            dog.move('x', x_compensation) 
            dog.turn(correction)    
            
            print(f"Sideways.. Cur:{current_yaw:>6.1f} | Err:{pid.Error:>5.1f} | CompX:{x_compensation}", end='\r')
            time.sleep(CONTROL_FREQ)
    finally:
        dog.stop()

# ================= 主程序执行 =================
if __name__ == "__main__":
    try:

        # 0. 锁定初始航向
        sum_y = 0
        for _ in range(15):
            sum_y += dog.read_yaw() 
            time.sleep(0.02)
        initial_yaw = sum_y / 15     
        print(f"航向锁定成功: {initial_yaw:.2f}")

        # 1. 梯形速度前进
        move_straight(speed=30, duration=9, target_yaw=initial_yaw)
        time.sleep(0.8)

        # 2. 180度闭环转向
        target_180 = initial_yaw + 180
        turn_to_angle(target_yaw=target_180)
        time.sleep(0.8)

        # 3. 横向移动
        move_sideways(speed_y=10, duration=4, target_yaw=target_180, x_compensation=2.4)
        time.sleep(0.5)
        move_sideways(speed_y=13, duration=5, target_yaw=target_180, x_compensation=2.4)
        time.sleep(0.5)
        move_sideways(speed_y=10, duration=4, target_yaw=target_180, x_compensation=2.4)
        time.sleep(1)

        # 4. 梯形速度前进
        move_straight(speed=20, duration=7, target_yaw=target_180)
        time.sleep(1) 

        # 5. 180度闭环转向
        turn_to_angle(target_yaw=initial_yaw)
        time.sleep(0.8)

        # 6. 横向移动
        move_sideways(speed_y=10, duration=4, target_yaw=initial_yaw, x_compensation=2.4)
        time.sleep(0.5)
        move_sideways(speed_y=13, duration=5, target_yaw=initial_yaw, x_compensation=2.4)
        time.sleep(0.5)
        move_sideways(speed_y=10, duration=4, target_yaw=initial_yaw, x_compensation=2.4)
        time.sleep(1)


        print("\n任务结束")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        dog.stop()