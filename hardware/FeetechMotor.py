import time
from . import macro
import scservo_sdk as scs
import math

# # motor zero position limits
# lower_limits_deg = [-126.05, -89.99, -89.99, -114.59, -179.99, -11.46]
# upper_limits_deg = [126.05, 101.46, 89.99, 103.13, 179.99, 114.59]

# motor2mujoco zero offset
offset_deg = [0.0, 90.0, -90.0, 0.0, 0.0, 0.0]

# motor2mujoco direction
direction = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# mujoco zero position limits
lower_limits_deg = [-126.05, -179.99, 0.0, -114.59, -179.99, -11.46]
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
        self.print_flag = False
    
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
        output = (constrained_angle + 180.0) * 4096.0 / 360.0
        output = round(output)

        return output, constrained_angle
    
    def _motor2deg(self, input_value):
        angle = (input_value * 360.0 / 4096.0) - 180.0 
        angle = angle - offset_deg[self.motor_id - 1]
        angle = angle * direction[self.motor_id - 1]
        return angle

    def setPosition(self, position):
        """
        位置范围：0~4095，对应-180~180度
        """
        # 转换为电机位置值
        conv_position, constrained_angle = self._deg2MotorLimited(position)
        if self.print_flag:
            print(f"电机ID {self.motor_id} 下发角度：{position}° 实际设置位置: {conv_position} 实际控制角度: {constrained_angle}°)")
        
        # 发送位置指令
        self.write_with_motor_ids(self.motor_id, "Goal_Position", conv_position)

    def getPosition(self):
        """
        位置范围：0~4095，对应-180~180度
        """
        position = self.read_with_motor_ids(self.motor_id, "Present_Position")
        conv_position = self._motor2deg(position)
        # print(f"Get position: {conv_position}")
        return conv_position

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
    motor = FeetechMotor(6, "/dev/ttyACM0")
    motor.connect()
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
        print(f"Current position: {current_position}, Goal position: {position}, Current_speed: {motor.getSpeed()}")
        time.sleep(0.005)
    motor.disconnect()


