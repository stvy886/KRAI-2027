import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from pymodbus.client import ModbusSerialClient
  
class Modbus(Node):
    def __init__(self, client):
        super().__init__("modbusY")

        self.client_ = client
        self.period = 0.01
        self.pub_ = self.create_publisher(UInt16, "process_encoderY", 10)
        self.timer = self.create_timer(self.period, self.timerCallback)

    def timerCallback(self):
        result_x_right = self.client_.read_input_registers(address=0, count=1, device_id=1)

        msg = UInt16()
        msg.data = result_x_right.registers[0]
        self.pub_.publish(msg)

def main():
    rclpy.init()
    
    master = ModbusSerialClient(
        port="/dev/modbus_2",
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
