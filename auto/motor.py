import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from pymodbus.client import ModbusSerialClient
import time
  
class Modbus(Node):
    def __init__(self, client):
        super().__init__("modbus")

        self.start_time = time.monotonic()
        self.client_ = client
        self.data = 1
        self.frequency = 0.01 
        self.pub_ = self.create_publisher(Int16, "test", 10)
        self.timer = self.create_timer(self.frequency, self.timerCallback)

    def timerCallback(self):

        elapsed = time.monotonic() - self.start_time
        if elapsed < 2.0:
            self.data = 1
        else:
            self.data = 0

        self.client_.write_register(address=0, value= self.data, slave=1)
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