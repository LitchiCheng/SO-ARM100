import sys
import os
import time
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

from feetech import FeetechMotorsBusConfig
from feetech import FeetechMotorsBus

class FeetechMotor:
    def __init__(self, motor_id, port="/dev/ttyACM0"):
        motors={
            # name: (index, model)
            "shoulder_pan": [1, "sts3215"],
            "shoulder_lift": [2, "sts3215"],
            "elbow_flex": [3, "sts3215"],
            "wrist_flex": [4, "sts3215"],
            "wrist_roll": [5, "sts3215"],
            "gripper": [6, "sts3215"],
        }
        self.motor_id = motor_id
        self.motors_bus = FeetechMotorsBus(FeetechMotorsBusConfig(
            port=port,
            motors=motors,
        ))
        self.motors_bus.connect()

    def setPosition(self, position):
        self.motors_bus.write_with_motor_ids(self.motors_bus.motor_models, self.motor_id, "Goal_Position", position)

    def getPosition(self):
        return self.motors_bus.read_with_motor_ids(self.motors_bus.motor_models, self.motor_id, "Present_Position")

    def close(self):
        self.motors_bus.disconnect()

def generatePositionSequence(start_position, range_value, loops=1):
    sequence = []
    for _ in range(loops):
        forward_positions = list(range(start_position, start_position + range_value + 1))
        sequence.extend(forward_positions)
        backward_positions = list(range(start_position + range_value - 1, start_position - 1, -1))
        sequence.extend(backward_positions)
    return sequence

if __name__ == "__main__":
    motor = FeetechMotor(6, "/dev/ttyACM0")
    motor.setPosition(2048)
    time.sleep(1)
    start_position = motor.getPosition()
    print(f"Start position: {start_position}")
    range_val = 600
    loop_count = 10
    result = generatePositionSequence(start_position, range_val, loop_count)
    for position in result:
        motor.setPosition(position)
        current_position = motor.getPosition()
        print(f"Current position: {current_position}, Goal position: {position}")
        time.sleep(0.005)
    motor.close()