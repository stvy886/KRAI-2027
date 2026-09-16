import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from pymodbus.client import ModbusSerialClient
import struct
  
class Modbus(Node):
    def __init__(self, client):
        super().__init__("modbus")

        self.client_ = client
        self.data = 1
        self.frequency = 1.0
        self.pub_ = self.create_publisher(Int16, "test", 10)

        self.timer = self.create_timer(self.frequency, self.timerCallback)
        self.client_.write_registers(address=0, value= self.data, slave=1)

    def timerCallback(self):
        msg = Int16()
        msg.data = self.data
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