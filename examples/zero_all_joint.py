import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm
import time

motor = fm.FeetechMotor(1, "/dev/ttyACM0")
motor.connect()
for i in range(1, 7):
    motor.setMotorId(i)
    motor.setSpeed(500)  # 设置目标速度为500步/秒
    start_timre = time.time()
    motor.setPosition(0)
    end_time = time.time()
    print(f"Motor ID {i} set to position 0 in {end_time - start_timre:.2f} seconds")
motor.disconnect()