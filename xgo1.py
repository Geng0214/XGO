"""
111.py - XGO 机器狗精确运动控制 + 视觉对准微调封装

功能:
  1. 精确前进 20cm（航向锁定 PID，保证走直线、距离准确）
  2. 精确左转 180°（陀螺仪 read_yaw 闭环反馈）
  3. 封装"前后左右微调"函数：视觉部分传入目标偏移参数后，
     通过小步横移/进退，使摄像头中心与目标中心几乎重合（仅封装函数）

参考 test.py 的增量式 PID、角度死区、最小输出阈值等思路。
"""

from xgolib import XGO
import time
import math

# ================= 调试参数 =================
KP = 1.2
KI = 0.05
KD = 0.5

# ---- 转向参数（两阶段：粗转 + 脉冲精调） ----
# read_yaw 每次约 0.5s（控制频率低），连续输出闭环每帧转角远超到位死区，
# 接近目标时必然过冲振荡。精调改为“脉冲-停稳-测量”的步进方式，不受读取延迟影响。
MIN_TURN_OUTPUT = 9      # 最小有效转向输出（电机死区之上，保证能转动；太小电机不转）
COARSE_ERR = 15          # 粗转结束阈值：误差小于该值进入脉冲精调
COARSE_MAX_OUT = 35      # 粗转最大输出（降低角速度，减小接近时的过冲）
PULSE_DEADBAND = 1.0     # 精调到位死区（角度）
MAX_PULSE = 25           # 精调最大脉冲次数（安全上限）
CAL_PULSE = 0.30         # 精调首脉冲：在线标定转向转速的时长 (s)
PULSE_FRAC = 0.85        # 精调每次脉冲目标转角 = 剩余误差 × 该系数
PULSE_MIN_T = 0.06       # 精调脉冲时长下限 (s)
PULSE_MAX_T = 0.45       # 精调脉冲时长上限 (s)
PULSE_STOP = 0.25        # 精调脉冲后停稳时间 (s)

# ---- 前进/横移锁向参数（脉冲式，修复直线跑偏与蛇形） ----
# 旧版增量式 PID：偏航误差小(2~3°)时输出=KP*err≈2.4，低于电机死区 → 转向电机不转，
# 无法纠偏，狗沿初始偏航方向越走越歪。现改为：偏航超死区时发一个短转向脉冲
# （约转过 2°），脉冲后即停，避免在读取延迟下连续输出造成的过冲蛇形。
STRAIGHT_DEADBAND = 1.5  # 锁向死区（角度）：偏航超过该值才发修正脉冲
MIN_STEER_OUTPUT = 9     # 锁向修正脉冲的输出（电机死区之上）
STEER_PULSE = 0.20       # 锁向修正脉冲时长 (s)，约对应转过 2°
CONTROL_FREQ = 0.04      # 前进/横移锁向控制周期

# ============ 距离标定参数（来自 xgolib 出厂标定） ============
# 运行时间 runtime = K * |距离(cm)| + MINTIME，速度恒定 → 保证距离准确
SPEED_X = 18            # 前进速度 (cm/s 量级)
SPEED_Y = 18            # 横移速度 (cm/s 量级)
DIST_K = 0.035          # 前进距离标定系数 (s/cm)
DIST_MINTIME = 0.55     # 前进最短运行时间 (s)
STRAFE_K = 0.0373       # 横移距离标定系数 (s/cm)
STRAFE_MINTIME = 0.5    # 横移最短运行时间 (s)

# ============ 视觉微调参数 ============
ALIGN_H_DEADBAND = 0.05   # 水平误差死区（归一化 -1~1），小于该值不动
ALIGN_V_DEADBAND = 0.05   # 垂直误差死区（归一化 -1~1）
ALIGN_STEP_CM = 1.5       # 单次微调步进距离 (cm)
ALIGN_MAX_ITER = 10       # 自动对准最大迭代次数（安全上限）
# ============================================

dog = XGO("xgomini")


# 串口偶发读取超时保护（pyserial 在 inWaiting 可读但 read 返回空时抛异常）
def safe_read_yaw(retries=3, delay=0.05):
    """安全读取 yaw，串口偶发超时/空读时自动重试。

    XGO 库的 read_yaw 在高频读写时偶发 pyserial SerialException
    （'device reports readiness to read but returned no data'）。
    这里捕获异常并重试，避免程序崩溃。
    """
    for attempt in range(retries):
        try:
            return dog.read_yaw()
        except Exception as e:
            if attempt == retries - 1:
                raise  # 重试耗尽，向上抛
            print(f"[警告] read_yaw 失败({e})，重试 {attempt + 1}/{retries}")
            time.sleep(delay)
    return 0.0


class IncrementalPID:
    """增量式 PID（用于前进/横移时锁定航向）"""
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


# ===================== 基础工具 =====================
def _norm_angle(a):
    """把任意角度归一化到 [-180, 180]，避免跨 0/±180 边界时绕圈"""
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


def lock_yaw(samples=15):
    """多次采样取平均，锁定当前航向角（返回归一化到 [-180,180]）"""
    s = 0
    for _ in range(samples):
        s += safe_read_yaw()
        time.sleep(0.02)
    return _norm_angle(s / samples)


def _move_with_yaw_lock(direction, speed, duration, target_yaw, x_compensation=0):
    """在指定方向(direction='x'/'y')上以恒定速度运动 duration 秒，
    过程中用【脉冲式锁向】保持航向走直线。x_compensation 用于横移时的 X 轴补偿。

    为什么用脉冲式而不是连续输出闭环：
      read_yaw 每次约 0.5s（控制频率低），若连续输出一个转向值，一次就转过
      5~6°（远超死区），从死区边缘必然过冲到另一侧，狗蛇形摇摆。
      改为：偏航误差超过死区时，发一个约转过 2° 的短转向脉冲，随即停止，
      下帧再测量。步进小、单次过冲可控，狗保持直线。
    """
    start_time = time.time()
    try:
        while time.time() - start_time < duration:
            current_yaw = _norm_angle(safe_read_yaw())
            error = _norm_angle(target_yaw - current_yaw)

            # 持续移动
            dog.move(direction, speed)
            if direction == 'y' and x_compensation:
                dog.move('x', x_compensation)

            # 偏航超死区 → 发一个短修正脉冲（约转过 2°），然后停
            if abs(error) > STRAIGHT_DEADBAND:
                dog.turn(int(MIN_STEER_OUTPUT if error > 0 else -MIN_STEER_OUTPUT))
                time.sleep(STEER_PULSE)
            dog.turn(0)
            time.sleep(CONTROL_FREQ)
    finally:
        dog.stop()


# ===================== 1. 精确前进 / 后退（按 cm） =====================
def move_forward_cm(distance_cm, speed=None, yaw=None):
    """精确前进指定厘米数（正数前进，负数后退）。
    基于出厂标定的运行时间公式 runtime = DIST_K*|cm| + DIST_MINTIME，
    运动全程用 PID 锁定航向，保证距离与方向都准确。

    Args:
        distance_cm: 前进距离(cm)，传负数则后退
        speed: 前进速度，默认 SPEED_X
        yaw: 目标航向，默认锁定当前航向
    """
    if speed is None:
        speed = SPEED_X
    if yaw is None:
        yaw = lock_yaw()
    runtime = DIST_K * abs(distance_cm) + DIST_MINTIME
    print(f"[前进] {distance_cm}cm, 速度 {speed}, 运行 {runtime:.2f}s, 航向 {yaw:.1f}")
    _move_with_yaw_lock('x', int(math.copysign(speed, distance_cm)), runtime, yaw)

def move_backward_cm(distance_cm, speed=None, yaw=None):
    """精确后退指定厘米数（等价于 move_forward_cm 传负数）"""
    move_forward_cm(-distance_cm, speed, yaw)


# ===================== 2. 精确左转 180° =====================
def turn_to_angle(target_yaw, timeout=20.0):
    """闭环转向到绝对目标航向。

    关键：用【累计角度坐标】计算误差，避免 180° 转向时在 ±180 边界抖动。

    为什么改两阶段（粗转 + 脉冲精调）：
      read_yaw 每次约 0.5s，控制频率低。旧版连续输出闭环在接近目标时，
      每帧转过角度（最小输出下约 5~6°）远超到位死区，必然过冲并来回振荡
      （第二次运行时 yaw 在目标附近 ±10° 摆不收敛）。
      现改为：
        1) 粗转：误差大时连续快速输出逼近到 COARSE_ERR 内；
        2) 精调：脉冲式步进——发一个短脉冲 → 停稳 → 测误差 → 再脉冲，
           单次转过角度小且每次停稳测量，不受读取延迟影响，稳定收敛到 ±1°。
      精调首次脉冲会在线标定当前转向转速，之后按“剩余误差×PULSE_FRAC”
      计算脉冲时长，自适应地面/电池差异。

    Returns:
        float: 最终航向角（归一化到 [-180, 180]）
    """
    # ---- 把归一化目标转成与当前累计 yaw 同坐标系的累计目标 ----
    start_raw = safe_read_yaw()              # 累计角度（可超过 ±180）
    start_norm = _norm_angle(start_raw)
    delta = target_yaw - start_norm         # 相对最短转角（±180 内）
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    target_abs = start_raw + delta          # 累计坐标系中的目标

    print(f"[转向] 目标角度: {target_yaw:.1f} (累计目标 {target_abs:.1f})")
    direction = 1.0
    checked = False
    prev_abs_err = None
    grow_count = 0

    try:
        # ================= 阶段1：粗转（连续输出，快速逼近） =================
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_raw = safe_read_yaw()
            error = target_abs - current_raw

            # 方向自检（只做一次）：累计坐标下方向正确则 |error| 单调减小，反之增大
            if not checked:
                if prev_abs_err is None:
                    prev_abs_err = abs(error)
                elif abs(error) > prev_abs_err + 8:
                    grow_count += 1
                else:
                    grow_count = 0
                if grow_count >= 5:  # 连续 5 周期误差都在变大 → 方向反
                    direction = -direction
                    checked = True
                    print("[转向] 检测到方向反，已自动反转")
                elif abs(error) <= prev_abs_err - 8:
                    checked = True

            if abs(error) <= COARSE_ERR:
                break

            out = direction * KP * error
            out = int(max(min(out, COARSE_MAX_OUT), -COARSE_MAX_OUT))
            if abs(out) < MIN_TURN_OUTPUT:
                out = MIN_TURN_OUTPUT if direction * error > 0 else -MIN_TURN_OUTPUT
            dog.turn(out)
            time.sleep(CONTROL_FREQ)

        dog.turn(0)
        dog.stop()
        time.sleep(0.4)   # 稳定后进入精调

        # ================= 阶段2：脉冲精调（不受读取延迟影响） =================
        yaw_speed = None   # 在线标定：out=MIN_TURN_OUTPUT 时转向转速 (°/s)
        for _ in range(MAX_PULSE):
            cur = safe_read_yaw()
            err = target_abs - cur
            if abs(err) <= PULSE_DEADBAND:
                break
            d = 1.0 if err > 0 else -1.0

            if yaw_speed is None:
                # 首脉冲：固定时长标定转速
                yaw0 = cur
                dog.turn(int(d * MIN_TURN_OUTPUT))
                time.sleep(CAL_PULSE)
                dog.turn(0)
                time.sleep(PULSE_STOP)
                yaw1 = safe_read_yaw()
                yaw_speed = abs(yaw1 - yaw0) / CAL_PULSE
                if yaw_speed < 1.0:
                    print(f"[精调] 警告: MIN_TURN_OUTPUT={MIN_TURN_OUTPUT} 几乎不转"
                          f"({yaw_speed:.1f}°/s)，请调大该参数")
                    yaw_speed = 1.0
                else:
                    print(f"[精调] 标定转向转速 {yaw_speed:.1f}°/s")
                continue

            # 按剩余误差计算脉冲时长，转过误差的 PULSE_FRAC 比例（避免过冲）
            pulse_t = max(PULSE_MIN_T, min(PULSE_MAX_T, abs(err) * PULSE_FRAC / yaw_speed))
            dog.turn(int(d * MIN_TURN_OUTPUT))
            time.sleep(pulse_t)
            dog.turn(0)
            time.sleep(PULSE_STOP)
            print(f"  [精调] err={err:6.1f} pulse={pulse_t:.2f}s")
    finally:
        dog.turn(0)
        dog.stop()
        time.sleep(0.4)   # 等待物理稳定，消除惯性
        # 回正：残余小偏角用短脉冲修正（最大 4 次）
        target_norm = _norm_angle(target_yaw)
        for _ in range(4):
            cur = _norm_angle(safe_read_yaw())
            err = _norm_angle(target_norm - cur)
            if abs(err) <= PULSE_DEADBAND:
                break
            d = 1.0 if err > 0 else -1.0
            dog.turn(int(d * MIN_TURN_OUTPUT))
            time.sleep(0.10)
            dog.turn(0)
            time.sleep(PULSE_STOP)
        dog.stop()
        time.sleep(0.3)
        final = _norm_angle(safe_read_yaw())  # 归一化到 [-180,180] 再返回/打印
        print(f"[转向] 完成, 最终角度: {final:.1f}")
        return final


def turn_left_180():
    """精确左转 180°：锁定当前航向后闭环转到 航向+180"""
    target = lock_yaw() + 180.0
    return turn_to_angle(target)


# ===================== 前后左右微调封装 =====================
def move_left_cm(distance_cm, speed=None, yaw=None, x_compensation=0):
    """精确左移指定厘米数（y 正方向为左）。
    若侧移过程中狗会前冲/后溜，可调整 x_compensation 抵消。"""
    if speed is None:
        speed = SPEED_Y
    if yaw is None:
        yaw = lock_yaw()
    runtime = STRAFE_K * abs(distance_cm) + STRAFE_MINTIME
    print(f"[左移] {distance_cm}cm, 速度 {speed}, 运行 {runtime:.2f}s")
    _move_with_yaw_lock('y', int(math.copysign(speed, distance_cm)), runtime, yaw,
                        x_compensation=x_compensation)

def move_right_cm(distance_cm, speed=None, yaw=None, x_compensation=0):
    """精确右移指定厘米数（y 负方向为右）"""
    move_left_cm(-distance_cm, speed, yaw, x_compensation)


def fine_tune_forward_cm(step=ALIGN_STEP_CM):
    """视觉对准用：前进一小步（默认 1.5cm）"""
    move_forward_cm(step)

def fine_tune_backward_cm(step=ALIGN_STEP_CM):
    """视觉对准用：后退一小步"""
    move_forward_cm(-step)

def fine_tune_left_cm(step=ALIGN_STEP_CM):
    """视觉对准用：左移一小步"""
    move_left_cm(step)

def fine_tune_right_cm(step=ALIGN_STEP_CM):
    """视觉对准用：右移一小步"""
    move_left_cm(-step)


# ===================== 主程序（演示功能 ） =====================
if __name__ == "__main__":
    try:
        # 0. 锁定初始航向
        initial_yaw = lock_yaw()
        print(f"航向锁定成功: {initial_yaw:.2f}")

        # 1. 精确前进3.3m
        move_forward_cm(330, yaw=initial_yaw)
        time.sleep(0.8)

        # 2. 精确左转 180°
        turn_left_180()
        time.sleep(0.8)
        # 转向后重新锁定当前航向（归一化到 ±180），作为横移锁向目标
        # （不直接用 turn_left_180 的返回值，避免累计角度坐标混乱）
        strafe_yaw = lock_yaw()
        print(f"横移锁向角度: {strafe_yaw:.1f}")

        # 3. 向左横移三段
        X_COMP = 2.1      # X 轴补偿（侧移时若后溜给正数，若前冲给负数，按实际调整）
        VISION_WAIT = 3.0 # 每次停下等待视觉识别的时间（秒）
        for dist in (50, 90, 50):
            move_left_cm(dist, yaw=strafe_yaw, x_compensation=X_COMP)
            time.sleep(VISION_WAIT)  # 等待视觉识别（暂时用 sleep 模拟）

        # 4. 前进（沿第一次转向后的航向，即回程方向）
        move_forward_cm(150, yaw=strafe_yaw)
        time.sleep(0.8)

        # 5. 转回初始航向 180°（turn_to_angle 内部自动归一化，避免绕圈）
        turn_to_angle(initial_yaw)
        time.sleep(0.8)
        # 转向后重新锁定当前航向，作为第二轮横移的锁向目标
        strafe_yaw2 = lock_yaw()
        print(f"横移锁向角度: {strafe_yaw2:.1f}")

        # 6. 向左横移三段
        for dist in (50, 90, 50):
            move_left_cm(dist, yaw=strafe_yaw2, x_compensation=X_COMP)
            time.sleep(VISION_WAIT)  # 等待视觉识别（暂时用 sleep 模拟）


        print("\n任务结束")

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        dog.stop()
