from xgolib import XGO
import time

# ================= 调试参数 =================
KP = 1.2
KI = 0.05
KD = 0.5

ANGLE_TOLERANCE = 1.5  # 角度死区：误差小于 1.5 度就不动了
MIN_TURN_OUTPUT = 8    # 最小输出阈值：PID 输出小于 8 时不发转向指令

CONTROL_FREQ = 0.04    # 25Hz

DISTANCE = 50          # 前进距离 (cm)
MOVE_SPEED = 18        # 前进速度（与官方 move_x_by 默认一致）
MOVE_K = 0.035         # 距离->时间 系数（官方标定值）
MOVE_MINTIME = 0.55    # 最小运行时间（官方标定值）

# ===== 功能4：缓慢向右横移 =====
SIDEWAYS_SPEED = -8        # 横移速度：负数=向右，正数=向左（低速缓慢横移）
SIDEWAYS_DURATION = 13      # 横移时长 (s)
SIDEWAYS_X_COMP = 2.4      # X轴漂移补偿（狗横移时前后漂移，需实机标定）
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
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        self.Error = error

        increment = (self.Kp * (self.Error - self.LastError) +
                     self.Ki * error +
                     self.Kd * (self.Error - 2 * self.LastError + self.LastLastError))

        self.PIDOutput += increment
        self.PIDOutput = max(min(self.PIDOutput, 50), -50)
        self.LastLastError = self.LastError
        self.LastError = self.Error
        return int(self.PIDOutput)


def read_current_yaw(samples=15, delay=0.02):
    """多次采样取平均，锁定当前航向角"""
    sum_y = 0
    for _ in range(samples):
        sum_y += dog.read_yaw()
        time.sleep(delay)
    return sum_y / samples


def turn_to_angle(target_yaw, timeout=8.0):
    """闭环转向到指定绝对角度（参考 test.py）"""
    pid = IncrementalPID(KP, KI, KD)
    start_time = time.time()
    arrival_count = 0

    print(f"\n[转向] 目标角度: {target_yaw:.1f}")

    try:
        while time.time() - start_time < timeout:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)

            if abs(pid.Error) < ANGLE_TOLERANCE:
                dog.move('x', 0)
                dog.turn(0)
                arrival_count += 1
            else:
                actual_turn = 0 if abs(correction) < MIN_TURN_OUTPUT else correction
                dog.move('x', 0)
                dog.turn(actual_turn)
                arrival_count = 0

            print(f"Turning.. Cur:{current_yaw:>6.1f} | Err:{pid.Error:>5.1f} | Out:{correction:>3}", end='\r')

            if arrival_count > 8:  # 连续约 0.3 秒稳定在范围内
                break

            time.sleep(CONTROL_FREQ)

    finally:
        dog.move('x', 0)
        dog.turn(0)
        dog.stop()
        time.sleep(0.5)  # 等机器人站稳，消除惯性
        print(f"\n转向完成,最终角度: {dog.read_yaw():.1f}")


def move_forward_distance(distance, target_yaw, speed=MOVE_SPEED):
    """
    前进指定距离 (cm)，同时用 PID 锁定航向。
    运行时间采用官方 move_x_by 的标定公式: t = k * |distance| + mintime
    """
    pid = IncrementalPID(KP, KI, KD)
    runtime = MOVE_K * abs(distance) + MOVE_MINTIME
    start_time = time.time()

    print(f"\n[前进] 距离: {distance}cm | 速度: {speed} | 航向锁定: {target_yaw:.1f} | 预计耗时: {runtime:.2f}s")

    try:
        while time.time() - start_time < runtime:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)

            dog.move('x', int(speed))
            dog.turn(correction)

            print(f"Forward.. Cur:{current_yaw:>6.1f} | Err:{pid.Error:>5.1f}", end='\r')
            time.sleep(CONTROL_FREQ)

    finally:
        dog.move('x', 0)
        dog.turn(0)
        dog.stop()
        print(f"\n前进完成,最终角度: {dog.read_yaw():.1f}")


def move_sideways(speed_y, duration, target_yaw, x_compensation=SIDEWAYS_X_COMP):
    """
    横移（参考 test.py）。speed_y 正数向左、负数向右。
    """
    pid = IncrementalPID(KP, KI, KD)
    start_time = time.time()

    print(f"\n[横移] 速度: {speed_y} | 航向锁定: {target_yaw:.1f} | "
          f"X轴补偿: {x_compensation} | 时长: {duration}s")

    try:
        while time.time() - start_time < duration:
            current_yaw = dog.read_yaw()
            correction = pid.Compute(target_yaw, current_yaw)

            dog.move('y', speed_y)          # 左右横移
            dog.move('x', x_compensation)   # 抵消横移时的前后漂移
            dog.turn(correction)            # 航向锁定

            print(f"Sideways.. Cur:{current_yaw:>6.1f} | Err:{pid.Error:>5.1f}", end='\r')
            time.sleep(CONTROL_FREQ)
    finally:
        dog.stop()


# ================= 主程序执行 =================
if __name__ == "__main__":
    try:
        # 1. 锁定当前航向角
        initial_yaw = read_current_yaw()
        print(f"当前航向锁定: {initial_yaw:.2f}")

        # 2. 根据当前航向，左转 90 度
        target_yaw = initial_yaw + 90
        turn_to_angle(target_yaw=target_yaw)
        time.sleep(0.8)

        # 3. 保持新航向，前进 20cm
        move_forward_distance(distance=DISTANCE, target_yaw=target_yaw)
        time.sleep(0.5)

        # 4. 降低速度慢慢向右横移
        move_sideways(speed_y=SIDEWAYS_SPEED, duration=SIDEWAYS_DURATION,
                      target_yaw=target_yaw)

        print("\n任务结束")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        dog.stop()
