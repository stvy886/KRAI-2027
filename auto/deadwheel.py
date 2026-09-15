import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray
import math

class Encoder():
    def __init__(self):
 
        self.ang = 0
        self.kuadran = 0
        self.last_kuadran = 0
        self.rotate = 0
        self.init_con = True
        self.init_con2 = True
        self.total_ang = 0
        self.diameter = 4.8

    def countAng(self, data):
        angle = data * 360.0/4096.0
        return angle

    def correctAng(self, init_ang):
        self.ang = self.ang - init_ang
        if (self.ang < 0):
            self.ang = self.ang + 360

    def rotation(self):
        if ((self.ang >= 0) & (self.ang <= 90)):
            self.kuadran = 1
        elif ((self.ang > 90) & (self.ang <= 180)):
            self.kuadran = 2
        elif ((self.ang > 180) & (self.ang <= 270)):
            self.kuadran = 3
        else:
            self.kuadran = 4

        if (self.init_con):
            self.last_kuadran = self.kuadran
            self.init_con = False

        if ((self.kuadran == 1) & (self.last_kuadran == 4)):
            self.rotate += 1
        elif ((self.kuadran == 4) & (self.last_kuadran == 1)):
            self.rotate -= 1

        self.last_kuadran = self.kuadran

    def countTotalAng(self):
        total_ang = self.ang + self.rotate*360
        return total_ang
    
    def countDistance(self):
        distance = ((self.diameter*math.pi)/360.0)*self.total_ang
        return distance
    
    def processedEnc(self, msg):
        if (self.init_con2):
            init_ang = self.countAng(msg)
            self.init_con2 = False

        self.ang = self.countAng(msg)
        self.correctAng(init_ang)
        self.rotation()
        self.total_ang = self.countTotalAng()
        distance = self.countDistance()

        return distance

class DeadWheel(Node):
    def __init__(self):
        super.__init__("deadwheel")

        self.frequency = 1.0
        self.sub_ = self.create_subscription(UInt16MultiArray, "process_encoder", self.encode, 10)
        self.pub_ = self.create_publisher(Float32MultiArray, "enc_val", 10)
        self.timer = self.create_timer(self.frequency, self.publishVal)

        self.xR = 0
        self.xL = 0
        self.y = 0

        self.encXR = Encoder()
        self.encXL = Encoder()
        self.encY = Encoder()

    def encode(self, msg):
        self.xR = self.encXR.processedEnc(msg[0])
        self.xL = self.encXL.processedEnc(msg[1])
        self.y = self.encY.processedEnc(msg[2])

    def publishVal(self):
        msg = Float32MultiArray()
        msg.data = [self.xR, self.xL, self.y]
        self.pub_.publish(msg)

def main():
    rclpy.init()
    enc= DeadWheel()
    rclpy.spin(enc)
    enc.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()