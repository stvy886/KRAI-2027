import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

class Odometri():
    def __init__(self, trck_width, forward_off, mecanum_rad, mecanum_lx, mecanum_ly, 
                 kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang):

        self.radius = mecanum_rad
        self.lx = mecanum_lx
        self.ly = mecanum_ly

        self.x = 0
        self.y = 0
        self.theta = 0

        self.frequency = 0

        self.prev_dist_x_right = 0
        self.prev_dist_x_left = 0
        self.prev_dist_y = 0
        
        self.trackwidth = trck_width
        self.forwardoffset = forward_off
        
        self.now_dist_x_right = 0
        self.now_dist_x_left = 0
        self.now_dist_y = 0

        self.kp_lin = kp_lin
        self.ki_lin = ki_lin
        self.kd_lin = kd_lin
        self.prev_err_lin = 0
        self.integral_lin = 0

        self.kp_ang = kp_ang
        self.ki_ang = ki_ang
        self.kd_ang = kd_ang
        self.prev_err_ang = 0
        self.integral_ang = 0

    def inputEncValue(self, R, L, S):
        self.now_dist_x_right = R
        self.now_dist_x_left = L
        self.now_dist_y = S

    def localFrame(self):
        dx_R = self.now_dist_x_right - self.prev_dist_x_right
        dx_L = self.now_dist_x_left - self.prev_dist_x_left
        dy = self.now_dist_y - self.prev_dist_y

        d_forward = (dx_R + dx_L)/2
        d_heading = (dx_R - dx_L)/self.trackwidth
        d_strafe = dy - (self.forwardoffset * d_heading)

        return d_forward, d_heading, d_strafe

    def globalFrame(self, d_forward, d_heading, d_strafe):
        theta_avg = self.theta + d_heading / 2.0

        dx_global = d_forward * math.cos(theta_avg) - d_strafe * math.sin(theta_avg)
        dy_global = d_forward * math.sin(theta_avg) + d_strafe * math.cos(theta_avg)

        self.x += dx_global
        self.y += dy_global
        self.theta += d_heading
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def updateVar(self):
        self.prev_dist_x_right = self.now_dist_x_right
        self.prev_dist_x_left = self.now_dist_x_left
        self.prev_dist_y = self.now_dist_y

    def pose(self):
        d_forward, d_heading, d_strafe = self.localFrame()
        self.globalFrame(d_forward, d_heading, d_strafe)
        self.updateVar()

    def PID(self, targetX, targetY, targetTheta, dt = 1):
        errorX = targetX - self.x
        errorY = targetY - self.y
        errorDist = math.hypot(errorX, errorY)

        errorTheta = math.atan2(math.sin(targetTheta - self.theta), 
                                math.cos(targetTheta - self.theta))

        self.integral_lin += errorDist * dt
        self.integral_ang += errorTheta * dt

        d_lin = (errorDist - self.prev_err_lin) / dt if dt > 0 else 0.0
        d_ang = (errorTheta - self.prev_err_ang) / dt if dt > 0 else 0.0
        self.prev_err_lin, self.prev_err_ang = errorDist, errorTheta

        v = (self.kp_lin * errorDist + self.ki_lin * self.integral_lin + self.kd_lin * d_lin)
        omega = (self.kp_ang * errorTheta + self.ki_ang * self.integral_ang + self.kd_ang * d_ang)

        v = np.clip(v, -0,6, 0,6)
        omega = np.clip(omega, -0.2, 0.2)

        heading_to_target = math.atan2(errorY, errorX)
        body_angle = heading_to_target - self.theta

        vx = v * math.cos(body_angle)
        vy = v * math.sin(body_angle)

        return vx, vy, omega
        
    def inverseKinematic(self, vx, vy, omega):
        L = self.lx + self.ly
        FL = (vx - vy - L * omega) / self.radius 
        FR = (vx + vy + L * omega) / self.radius 
        BL = (vx + vy - L * omega) / self.radius
        BR = (vx - vy + L * omega) / self.radius

        return FL, FR, BL, BR

    def targetToRpm(self, targetX, targetY, targetTheta):
        vx, vy, omega = self.PID(targetX, targetY, targetTheta)
        FL, FR, BL, BR = self.inverseKinematic(vx, vy, omega)

        return FL, FR, BL, BR
    
    def rpmToPwm(self, rpm, rpm_max, pwm_max):
        pwm = (rpm/rpm_max)*pwm_max
        pwm = np.clip(pwm, -pwm_max, pwm_max)

        return pwm

class Navigation(Node):
    def __init__(self, track_width, forward_offset, radius, lx, ly, 
                 kp_lin = 2.0, ki_lin = 0, kd_lin = 1.0, 
                 kp_ang = 2.0, ki_ang = 0, kd_ang = 1.0):
        super().__init__("odom_subs")

        self.robot = Odometri(track_width, forward_offset, radius, lx, ly, 
                              kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang)
        self.subs = self.create_subscription(Float32MultiArray, 
                                             "modbus_com", 
                                             self.navigation, 
                                             10)

        self.pub_ = self.create_publisher(Float32MultiArray, "pose", 10)
        self.timer_ = self.create_timer(1.0, self.timerCallback)

    def navigation(self, msg):
        self.robot.inputEncValue(msg[0], msg[1], msg[2])
        self.robot.pose()

    def timerCallback(self):
        msg = Float32MultiArray()
        msg.data = [self.robot.x, self.robot.y, self.robot.theta]
        self.pub_.publish(msg)

def main():
    rclpy.init()
    node = Navigation(10, 5, 0.25, 5, 5)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()
