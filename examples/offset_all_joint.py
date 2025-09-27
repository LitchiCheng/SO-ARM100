import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm

for i in range(6):
    motor = fm.FeetechMotor(i+1, "/dev/ttyACM0")
    motor.connect()
    motor.setOffsetCurrent()