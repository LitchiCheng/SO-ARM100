import time
import macro
import scservo_sdk as scs

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
        self.motor_id = motor_id
        self.port = port
        
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

    def setPosition(self, position):
        self.write_with_motor_ids(self.motor_id, "Goal_Position", position)

    def getPosition(self):
        return self.read_with_motor_ids(self.motor_id, "Present_Position")

def generatePositionSequence(start_position, range_value, loops=1):
    sequence = []
    for _ in range(loops):
        forward_positions = list(range(start_position, start_position + range_value + 1))
        sequence.extend(forward_positions)
        backward_positions = list(range(start_position + range_value - 1, start_position - 1, -1))
        sequence.extend(backward_positions)
    return sequence

if __name__ == "__main__":
    motor = FeetechMotor(5, "/dev/ttyACM0")
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
        print(f"Current position: {current_position}, Goal position: {position}")
        time.sleep(0.005)
    motor.disconnect()