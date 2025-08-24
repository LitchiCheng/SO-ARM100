import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm

motor = fm.FeetechMotor(1, "/dev/ttyACM0")
motor.connect()
for i in range(1, 7):
    motor.setMotorId(i)
    print(f"Motor {i} position: {motor.getPosition()}")
motor.disconnect()