
import zmq
import json
import sys
import os
import time
import math
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm

from typing import Any
import numpy as np
import rerun as rr

motors_key=[
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
]

def _init_rerun(session_name: str = "lerobot_control_loop") -> None:
    """Initializes the Rerun SDK for visualizing the control loop."""
    batch_size = os.getenv("RERUN_FLUSH_NUM_BYTES", "8000")
    os.environ["RERUN_FLUSH_NUM_BYTES"] = batch_size
    rr.init(session_name)
    memory_limit = os.getenv("LEROBOT_RERUN_MEMORY_LIMIT", "10%")
    rr.spawn(memory_limit=memory_limit)

def log_rerun_data(observation: dict[str | Any], action: dict[str | Any]):
    for obs, val in observation.items():
        if isinstance(val, float):
            rr.log(f"observation.{obs}", rr.Scalars(val))
        elif isinstance(val, np.ndarray):
            if val.ndim == 1:
                for i, v in enumerate(val):
                    rr.log(f"observation.{obs}_{i}", rr.Scalars(float(v)))
            else:
                rr.log(f"observation.{obs}", rr.Image(val), static=True)
    for act, val in action.items():
        if isinstance(val, float):
            rr.log(f"action.{act}", rr.Scalars(val))
        elif isinstance(val, np.ndarray):
            for i, v in enumerate(val):
                rr.log(f"action.{act}_{i}", rr.Scalars(float(v)))

# 初始化机械臂
motor = fm.FeetechMotor(1, "/dev/ttyACM0")
motor.connect()

# # ZMQ配置
# ZMQ_IP = "127.0.0.1"
# ZMQ_PORT = "5555"
# context = zmq.Context()
# socket = context.socket(zmq.SUB)
# socket.connect(f"tcp://{ZMQ_IP}:{ZMQ_PORT}")
# # 订阅所有消息（空字符串表示订阅所有主题）
# socket.setsockopt_string(zmq.SUBSCRIBE, "")

# print(f"等待仿真端发布数据... 连接到：tcp://{ZMQ_IP}:{ZMQ_PORT}")

# ZMQ IPC配置 - 使用进程间通信协议
ZMQ_IPC_PATH = "ipc:///tmp/robot_arm_comm.ipc"  # 与发布者相同的IPC路径
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(ZMQ_IPC_PATH)  # 连接到IPC路径
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息

print(f"控制端订阅者启动，连接到：{ZMQ_IPC_PATH}")
_init_rerun()

try:
    while True:
        data = socket.recv_string().strip()
        if not data:
            continue
        action = json.loads(data)
        joint_pos_deg = [val for key, val in action.items() if key.endswith(".pos")]
        joint_now_pos = []
        # 发送控制指令给实际机械臂
        for i in range(1, 7):
            motor.setMotorId(i)
            motor.setSpeed(1000)  # 设置目标速度为1000步/秒
            motor.printFlag(False)  # 打开打印
            if i == 1 or i == 2:
                motor.setPID(16, 16, 0)
            elif i == 5:
                motor.setPID(16, 16, 0)
            else:   
                motor.setPID(32, 32, 0)  # 设置PID参数
            motor.setPosition(math.degrees(joint_pos_deg[i - 1]))
            joint_now_pos.append(math.radians(motor.getPosition()))
        
        observation = {f"{motor}.pos": val for motor, val in zip(motors_key, joint_now_pos)}
        log_rerun_data(observation, action)
        
except KeyboardInterrupt:
    print("程序被用户中断")
except Exception as e:
    print(f"接收/控制错误：{e}")
finally:
    # 关闭连接与机械臂
    socket.close()
    context.term()
    motor.disconnect()
