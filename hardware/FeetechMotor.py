import time
from . import macro
import scservo_sdk as scs
import math

# # motor middle position limits
# offset_deg = [0.0, -10.0, 0.0, 0.0, 0.0, 0.0]
# direction = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
# lower_limits_deg = [-126.05, -89.99, -89.99, -114.59, -179.99, -11.46]
# upper_limits_deg = [126.05, 101.46, 89.99, 103.13, 179.99, 114.59]
# lower_limits_deg = [-180.0, -180.0, -180.0, -180.0, -180.0, -180.0]
# upper_limits_deg = [180.0, 180.0, 180.0, 180.0, 180.0, 180.0]

# motor2mujoco zero offset
offset_deg = [0.0, 80.0, -90.0, 0.0, 0.0, 0.0]
direction = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0] # motor2mujoco direction
lower_limits_deg = [-126.05, -179.99, 0.0, -114.59, -179.99, -11.46] # mujoco zero position limits
upper_limits_deg = [126.05, 11.46, 179.99, 103.13, 179.99, 114.59]

def convert_to_bytes(value, bytes):
    if bytes == 1:
        data = [
            scs.SCS_LOBYTE(scs.SCS_LOWORD(value)),
        ]
    elif bytes == 2:
        data = [
            scs.SCS_LOBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_LOWORD(value)),
        ]
    elif bytes == 4:
        data = [
            scs.SCS_LOBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_LOWORD(value)),
            scs.SCS_LOBYTE(scs.SCS_HIWORD(value)),
            scs.SCS_HIBYTE(scs.SCS_HIWORD(value)),
        ]
    else:
        raise NotImplementedError(
            f"Value of the number of bytes to be sent is expected to be in [1, 2, 4], but "
            f"{bytes} is provided instead."
        )
    return data

class FeetechMotor:
    def __init__(self, motor_id, port="/dev/ttyACM0"):
        self.setMotorId(motor_id)
        self.port = port
        self.print_flag = True
    
    def printFlag(self, on):
        if on:
            self.print_flag = True
        else:
            self.print_flag = False

    def setMotorId(self, motor_id):
        if not isinstance(motor_id, int):
            raise TypeError("Motor ID must be an integer.")
        if motor_id < 1 or motor_id > 6:
            raise ValueError("Motor ID must be between 1 and 6.")
        self.motor_id = motor_id
        
    def connect(self):
        self.port_handler = scs.PortHandler(self.port)
        self.packet_handler = scs.PacketHandler(macro.PROTOCOL_VERSION)
        try:
            if not self.port_handler.openPort():
                raise OSError(f"Failed to open port '{self.port}'.")
        except Exception:
            print("choose right port!")
            raise

        self.is_connected = True
        self.port_handler.setPacketTimeoutMillis(macro.TIMEOUT_MS)
    
    def disconnect(self):
        if self.port_handler is not None:
            self.port_handler.closePort()
            self.port_handler = None

        self.packet_handler = None
        self.is_connected = False
    
    def set_bus_baudrate(self, baudrate):
        present_bus_baudrate = self.port_handler.getBaudRate()
        if present_bus_baudrate != baudrate:
            print(f"Setting bus baud rate to {baudrate}. Previously {present_bus_baudrate}.")
            self.port_handler.setBaudRate(baudrate)

            if self.port_handler.getBaudRate() != baudrate:
                raise OSError("Failed to write bus baud rate.")

    def read_with_motor_ids(self, motor_ids, data_name, num_retry=macro.NUM_READ_RETRY):
        return_list = True
        if not isinstance(motor_ids, list):
            return_list = False
            motor_ids = [motor_ids]

        addr, bytes = macro.SCS_SERIES_CONTROL_TABLE[data_name]
        group = scs.GroupSyncRead(self.port_handler, self.packet_handler, addr, bytes)
        for idx in motor_ids:
            group.addParam(idx)

        for _ in range(num_retry):
            comm = group.txRxPacket()
            if comm == scs.COMM_SUCCESS:
                break

        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(
                f"Read failed due to communication error on port {self.port_handler.port_name} for indices {motor_ids}: "
                f"{self.packet_handler.getTxRxResult(comm)}"
            )

        values = []
        for idx in motor_ids:
            value = group.getData(idx, addr, bytes)
            values.append(value)

        if return_list:
            return values
        else:
            return values[0]
    
    def write_with_motor_ids(self, motor_ids, data_name, values, num_retry=macro.NUM_WRITE_RETRY):
        if not isinstance(motor_ids, list):
            motor_ids = [motor_ids]
        if not isinstance(values, list):
            values = [values]

        addr, bytes = macro.SCS_SERIES_CONTROL_TABLE[data_name]
        group = scs.GroupSyncWrite(self.port_handler, self.packet_handler, addr, bytes)
        for idx, value in zip(motor_ids, values, strict=True):
            data = convert_to_bytes(value, bytes)
            group.addParam(idx, data)

        for _ in range(num_retry):
            comm = group.txPacket()
            if comm == scs.COMM_SUCCESS:
                break

        if comm != scs.COMM_SUCCESS:
            raise ConnectionError(
                f"Write failed due to communication error on port {self.port_handler.port_name} for indices {motor_ids}: "
                f"{self.packet_handler.getTxRxResult(comm)}"
            )

    def _deg2MotorLimited(self, input_value):
        input_value = input_value * direction[self.motor_id - 1]
        # 获取当前电机的角度限位
        min_angle = lower_limits_deg[self.motor_id - 1 ]
        max_angle = upper_limits_deg[self.motor_id - 1]
    
        constrained_angle = max(min(input_value, max_angle), min_angle)
        if constrained_angle != input_value:
            if self.print_flag:
                print(f"警告：位置 {input_value} 超出限位范围 [{min_angle}, {max_angle}]，已修正为 {constrained_angle}")
        constrained_angle = constrained_angle + offset_deg[self.motor_id - 1]
        # 公式：位置 = 中间值(2048) + 角度 × (总步数4096 / 总角度360)
        output_pos = 2048 + constrained_angle * (4096 / 360)
        # 6. 确保位置在电机硬件范围内（0~4095）
        output_pos = max(0, min(round(output_pos), 4095))

        return output_pos, constrained_angle
    
    def _motor2deg(self, input_pos):
        """
        电机硬件位置（0~4095）→ 实际角度（-180°~180°）
        """
        # 1. 位置→原始角度（基于2048对应0°）
        raw_angle = (input_pos - 2048) * (360 / 4096)
        # 2. 抵消offset补偿
        raw_angle -= offset_deg[self.motor_id - 1]
        # 3. 应用方向修正（与角度→位置的方向一致）
        raw_angle = raw_angle * direction[self.motor_id - 1]
        return round(raw_angle, 2)  # 保留2位小数，避免精度冗余

    def setPosition(self, position_deg):
        """
        设定舵机角度（用户预期范围：-180° ~ 180°）
        position_deg: 目标角度（单位：度）
        """
        # 转换角度为电机硬件位置（0~4095），并应用限位
        conv_position, constrained_angle = self._deg2MotorLimited(position_deg)
        if self.print_flag:
            print(f"电机ID {self.motor_id} 下发角度：{position_deg}° → 修正后角度：{constrained_angle}° → 电机位置：{conv_position}")
        
        # 发送位置指令到舵机
        self.write_with_motor_ids(self.motor_id, "Goal_Position", conv_position)

    def setRawPosition(self, position):
        self.write_with_motor_ids(self.motor_id, "Goal_Position", position)

    def getPosition(self):
        """
        位置范围：0~4095，对应-180~180度
        """
        position = self.read_with_motor_ids(self.motor_id, "Present_Position")
        conv_position = self._motor2deg(position)
        # print(f"Get position: {conv_position}")
        return conv_position

    def getRawPosition(self):
        """
        位置范围：0~4095
        """
        position = self.read_with_motor_ids(self.motor_id, "Present_Position")
        return position
    
    def getOffset(self):
        return self.read_with_motor_ids(self.motor_id, "Offset")

    def setOffsetCurrent(self):
        self.write_with_motor_ids(self.motor_id, "Offset", 0)
        time.sleep(1.0)
        actual_positions = self.getRawPosition()
        print(f"Actual positions: {actual_positions}")
        homing_offset = actual_positions - int(macro.MODEL_RESOLUTION / 2)
        if homing_offset < 0:
            homing_offset = 0
            print(f"Homing offset is negative, set to 0. Check middle position of motor.")
        else:
            print(f"Homing offset: {homing_offset}")
        self.write_with_motor_ids(self.motor_id, "Offset", homing_offset)
        time.sleep(1.0)
        self.write_with_motor_ids(self.motor_id,"Min_Angle_Limit", 0)
        self.write_with_motor_ids(self.motor_id,"Max_Angle_Limit", 4095)

    def getAllConfig(self):
        config = {}
        for key in macro.SCS_SERIES_CONTROL_TABLE.keys():
            try:
                value = self.read_with_motor_ids(self.motor_id, key)
                config[key] = value
                print(f"{key}: {value}")
            except Exception as e:
                print(f"读取配置项 {key} 时出错: {e}")
        return config
    
    def setSpeed(self, speed):
        """
        步/s	单位时间（每秒）内运动的步数
        """
        self.write_with_motor_ids(self.motor_id, "Goal_Speed", speed)

    def setOffset(self, offset):
        self.write_with_motor_ids(self.motor_id, "Offset", offset)

    def getSpeed(self):
        return self.read_with_motor_ids(self.motor_id, "Present_Speed")
    
    def setTorqueEnable(self, enable):
        """
        写0：关闭扭力输出/自由状态； 写1：打开扭力输出； 写2：阻尼状态
        """
        self.write_with_motor_ids(self.motor_id, "Torque_Enable", enable)

    def setPID(self, p, i, d):
        self.write_with_motor_ids(self.motor_id, "P_Coefficient", p)
        self.write_with_motor_ids(self.motor_id, "I_Coefficient", i)
        self.write_with_motor_ids(self.motor_id, "D_Coefficient", d)

def generatePositionSequence(start_position, range_value, loops=1):
    sequence = []
    for _ in range(loops):
        forward_positions = list(range(start_position, start_position + range_value + 1))
        sequence.extend(forward_positions)
        backward_positions = list(range(start_position + range_value - 1, start_position - 1, -1))
        sequence.extend(backward_positions)
    return sequence

if __name__ == "__main__":
    pass


