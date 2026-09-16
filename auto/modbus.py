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

    def timerCallback(self):
        result_x_right = self.client_.read_input_registers(address=0, count=1, slave=1)
        result_x_left = self.client_.read_input_registers(address=0, count=1, slave=2)
        result_y = self.client_.read_input_registers(address=0, count=1, slave=3)

        msg = UInt16MultiArray()
        msg.data = [result_x_right[0], result_x_left[0], result_y[0]]
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
