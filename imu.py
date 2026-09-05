# 导入XGO库
from xgolib import XGO
import time

class RobotDogSensors:
    def __init__(self):
        self.dog = XGO("xgolite")

    def read_pose(self):
        """
        读取并返回机器狗的位姿角度。
        """
        roll = self.dog.read_roll()
        pitch = self.dog.read_pitch()
        yaw = self.dog.read_yaw()
        return roll, pitch, yaw

# 创建传感器对象
sensors = RobotDogSensors()

# 读取位姿
while True:
	
	roll, pitch, yaw = sensors.read_pose()
	print(f"Roll={roll}, Pitch={pitch}, Yaw={yaw}")
	time.sleep(1)
    