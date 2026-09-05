"""机器狗运动控制封装"""

import time


class DogController:
    def __init__(self, dog):
        self.dog = dog

    def walk_forward(self, speed=12):
        self.dog.move('x', speed)

    def walk_backward(self, speed=12):
        self.dog.move('x', -speed)

    def turn_left(self, speed=10):
        self.dog.turn(speed)

    def turn_right(self, speed=10):
        self.dog.turn(-speed)

    def strafe_left(self, speed=10):
        self.dog.move('y', speed)

    def strafe_right(self, speed=10):
        self.dog.move('y', -speed)

    def stop(self):
        self.dog.stop()

    def reset(self):
        self.dog.reset()
        time.sleep(1)

    def set_attitude(self, pitch=0, roll=0, yaw=0):
        dirs, vals = [], []
        if pitch != 0:
            dirs.append('p'); vals.append(pitch)
        if roll != 0:
            dirs.append('r'); vals.append(roll)
        if yaw != 0:
            dirs.append('y'); vals.append(yaw)
        if dirs:
            self.dog.attitude(dirs, vals)

    def set_height(self, z=90):
        self.dog.translation('z', z)
