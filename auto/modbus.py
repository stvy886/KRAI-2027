import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from pymodbus.client import ModbusSerialClient
import struct
  
class Modbus(Node):
    def __init__(self, client):
        super().__init__("modbus")

        self.client_ = client
        self.frequency = 1.0
        self.pub_ = self.create_publisher(UInt16MultiArray, "process_encoder", 10)

        self.timer = self.create_timer(self.frequency, self.timerCallback)

    def convertToFloat(self, low, high):
        combined = (high << 16) | low
        return struct.unpack('<f', struct.pack('<I', combined))[0]

    def timerCallback(self):
        result_x_right = self.client_.read_input_registers(address=0, count=2, slave=1)
        result_x_left = self.client_read_input_registers(address=0, count=2, slave=2)
        result_y = self.client_.read_input_registers(addrees=0, count=2, slave=3)

        distance_x_right = self.convertToFloat(result_x_right.registers[0], result_x_right.registers[1])
        distance_x_left = self.convertToFloat(result_x_left.registers[0], result_x_left.registers[1])
        distance_y = self.convertToFloat(result_y.registers[0], result_y.registers[1])

        msg = UInt16MultiArray()
        msg.data = [distance_x_right, distance_x_left, distance_y]
        self.pub_.publish(msg)


def main():
    rclpy.init()
    
    master = ModbusSerialClient(
        port="/dev/ttyUSB0",
        baudrate=115200,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=1
    )
    
    master.connect()

    modbusPub = Modbus(master)
    rclpy.spin(modbusPub)
    modbusPub.destroy_node()
    master.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()