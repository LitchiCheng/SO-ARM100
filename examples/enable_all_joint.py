import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm

motor = fm.FeetechMotor(1, "/dev/ttyACM0")
motor.connect()
print("Torque enable state before:")
for i in range(1, 7):
    motor.setMotorId(i)
    print(f"  Motor {i}: {motor.getTorqueEnable()}")
for i in range(1, 7):
    motor.setMotorId(i)
    motor.setTorqueEnable(1)
print("Torque enable state after:")
for i in range(1, 7):
    motor.setMotorId(i)
    print(f"  Motor {i}: {motor.getTorqueEnable()}")
motor.disconnect()
