import zmq
import json
import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm

import time

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

# 接收并解析数据
try:
    while True:
        # 接收数据
        data = socket.recv_string().strip()
        if not data:
            continue
        
        # 解析JSON数据
        json_data = json.loads(data)
        joint_pos_deg = json_data["joint_pos"]
        control_mode = json_data["control_mode"]
        
        # 发送控制指令给实际机械臂
        if control_mode == "position":
            for i in range(1, 7):
                motor.setMotorId(i)
                motor.setSpeed(2000)  # 设置目标速度为1000步/秒
                motor.printFlag(False)  # 打开打印
                if i == 1 or i == 2:
                    motor.setPID(16, 16, 0)
                else:   
                    motor.setPID(32, 32, 0)  # 设置PID参数
                motor.setPosition(joint_pos_deg[i - 1])
                # print(f"Motor ID {i} set to position {joint_pos_deg[i - 1]}°")
        
        # 短暂延迟，避免CPU占用过高
        # time.sleep(0.01)
        
except KeyboardInterrupt:
    print("程序被用户中断")
except Exception as e:
    print(f"接收/控制错误：{e}")
finally:
    # 关闭连接与机械臂
    socket.close()
    context.term()
    motor.disconnect()
