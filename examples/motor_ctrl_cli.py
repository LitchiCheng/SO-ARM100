import sys
import os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)
from hardware import FeetechMotor as fm

import time

def print_usage():
    """打印使用帮助信息"""
    print("Feetech电机控制工具")
    print("使用方法:")
    print("  设置位置: python motor_control.py <电机ID> set_pos <目标角度>")
    print("  获取位置: python motor_control.py <电机ID> get_pos")
    print("  启用扭矩: python motor_control.py <电机ID> enable")
    print("  禁用扭矩: python motor_control.py <电机ID> disable")
    print("  设置速度: python motor_control.py <电机ID> set_speed <目标速度>")
    print("  获取速度: python motor_control.py <电机ID> get_speed")

if __name__ == "__main__":
    # 检查命令行参数合法性
    if len(sys.argv) < 3:
        print_usage()
        sys.exit(1)

    try:
        motor_id = int(sys.argv[1])
        operation = sys.argv[2].lower()  # 转为小写确保不区分大小写
        if motor_id < 1 or motor_id > 6:
            print("错误: 电机ID必须在1到6之间")
            sys.exit(1)
        motor = fm.FeetechMotor(motor_id, "/dev/ttyACM0")
        motor.connect()
        if operation == "set_pos":
            if len(sys.argv) != 4:
                print("set操作需要指定目标角度: python motor_control.py <ID> set_pos <角度>")
                sys.exit(1)
            target_value = float(sys.argv[3])
            motor.setPosition(target_value)
        elif operation == "get_pos":
            position = motor.getPosition()
            print(f"电机ID {motor_id} 的当前位置: {position}°")
        elif operation == "set_speed":
            if len(sys.argv) != 4:
                print("set_speed操作需要指定目标速度: python motor_control.py <ID> set_speed <速度>")
                sys.exit(1)
            speed_value = int(sys.argv[3])
            motor.setSpeed(speed_value)
        elif operation == "get_speed":
            speed = motor.getSpeed()
            print(f"电机ID {motor_id} 的当前速度: {speed} 步/秒")
        elif operation == "enable":
            motor.setTorqueEnable(1)
            print(f"电机ID {motor_id} 的扭矩已启用")
        elif operation == "disable":
            motor.setTorqueEnable(0)
            print(f"电机ID {motor_id} 的扭矩已禁用")
        elif operation == "damp":
            motor.setTorqueEnable(2)
            print(f"电机ID {motor_id} 已设置为阻尼状态")
        elif operation == "get_config":
            motor.getAllConfig()
        elif operation == "set_pid":
            if len(sys.argv) != 6:
                print("set_pid操作需要指定P、I、D值: python motor_control.py <ID> set_pid <P> <I> <D>")
                sys.exit(1)
            p_value = int(sys.argv[3])
            i_value = int(sys.argv[4])
            d_value = int(sys.argv[5])
            motor.setPID(p_value, i_value, d_value)
            print(f"电机ID {motor_id} 的PID已设置为 P={p_value}, I={i_value}, D={d_value}")
        else:
            print(f"未知操作: {operation}")
            print_usage()
            motor.disconnect()
            sys.exit(1)
        motor.disconnect()
            
    except ValueError:
        print("错误: 电机ID和角度值必须是数字")
        motor.disconnect()
        sys.exit(1)
    except IndexError:
        print_usage()
        motor.disconnect()
        sys.exit(1)
