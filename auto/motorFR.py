import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymodbus.client import ModbusSerialClient

class Modbus(Node):
    def __init__(self, client):
        super().__init__("modbusFR")

        self.client_ = client
        self.sub_ = self.create_subscription(Float32, "motorFR", self.pwmCallback, 10)

    def pwmCallback(self, msg):
        pwm = int(msg.data)
        self.client_.write_register(address=0, value=(pwm & 0xffff), slave=1)

def main():
    rclpy.init()
    
    master = ModbusSerialClient(
        port="/dev/ttyUSB5",
        baudrate=115200,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=1
    )
    
    master.connect()

    modbusSub = Modbus(master)
    rclpy.spin(modbusSub)
    modbusSub.destroy_node()
    master.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()