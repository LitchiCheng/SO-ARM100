import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm
import time

# 目标位置（角度）
motor_positions = [
    0.17,                  
    -180,
    170,
    67,
    0,
    0
]

# 允许的位置误差（度）
ANGLE_TOLERANCE = 3.0

motor = fm.FeetechMotor(1, "/dev/ttyACM0")
motor.connect()

try:
    # 设置所有电机的目标位置和速度
    for i in range(1, 7):
        motor.setMotorId(i)
        motor.setSpeed(500)  # 设置目标速度为500步/秒
        target_angle = motor_positions[i - 1]
        motor.setPosition(target_angle)  # 直接使用角度设置位置
        print(f"电机 {i} 已设置目标位置: {target_angle}°")

    # 等待所有电机到达目标位置
    all_in_position = False
    timeout = 10  # 超时时间（秒）
    start_time = time.time()
    
    print("等待所有电机到达目标位置...")
    while not all_in_position and (time.time() - start_time) < timeout:
        all_in_position = True  # 先假设所有电机都已到位
        
        for i in range(1, 7):
            motor.setMotorId(i)
            current_angle = motor.getPosition()  # 直接获取当前角度
            target_angle = motor_positions[i - 1]
            angle_diff = abs(current_angle - target_angle)
            
            # 检查是否在允许的误差范围内
            if angle_diff > ANGLE_TOLERANCE:
                all_in_position = False
                print(f"电机 {i}: 当前 {current_angle:.2f}°，目标 {target_angle}°，误差 {angle_diff:.2f}°")
        
        if not all_in_position:
            time.sleep(0.1)  # 短暂延迟后再次检查

    if all_in_position:
        print("所有电机已到达目标位置")
    else:
        print(f"警告：等待超时（{timeout}秒），部分电机未完全到位")

    # 禁用所有电机扭矩
    for i in range(1, 7):
        motor.setMotorId(i)
        motor.setTorqueEnable(0)
    print("所有电机已禁用扭矩")

finally:
    motor.disconnect()
    print("已断开连接")
