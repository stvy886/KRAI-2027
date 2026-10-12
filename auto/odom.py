import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float32, Float32MultiArray
import math
import numpy as np

class Odometri(Node):
    def __init__(self, trck_width, forward_off, mecanum_rad, mecanum_lx, mecanum_ly, 
                 kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang, dt):

        super().__init__('odometry')

        self.init_state = 0
        self.first_con = True

        # variable for inverse kinematic
        self.radius = mecanum_rad
        self.lx = mecanum_lx
        self.ly = mecanum_ly

        # global pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # variable for local pose
        self.prev_dist_x_right = 0.0
        self.prev_dist_x_left = 0.0
        self.prev_dist_y = 0.0
        
        self.trackwidth = trck_width
        self.forwardoffset = forward_off

        # distance input from encoder
        self.now_dist_x_right = 0.0
        self.now_dist_x_left = 0.0
        self.now_dist_y = 0.0

        # PID variable for linear velocity
        self.kp_lin = kp_lin
        self.ki_lin = ki_lin
        self.kd_lin = kd_lin
        self.prev_err_lin = 0.0
        self.integral_lin = 0.0

        # PID variable for angular velocity
        self.kp_ang = kp_ang
        self.ki_ang = ki_ang
        self.kd_ang = kd_ang
        self.prev_err_ang = 0.0
        self.integral_ang = 0.0

        self.dt = dt

        # Publisher
        self.pubPose_ = self.create_publisher(Float32MultiArray, "pose", 10)
        self.pubFR_ = self.create_publisher(Float32, "motorFR", 10)
        self.pubFL_ = self.create_publisher(Float32, "motorFL", 10)
        self.pubBR_ = self.create_publisher(Float32, "motorBR", 10)
        self.pubBL_ = self.create_publisher(Float32, "motorBL", 10)

    def inputEncValue(self, R, L, S):
        self.now_dist_x_right = R
        self.now_dist_x_left = L
        self.now_dist_y = S

        if self.first_con:
            self.prev_dist_x_right = R
            self.prev_dist_x_left = L
            self.prev_dist_y = S
            self.first_con = False

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

    def PID(self, targetX, targetY, targetTheta):
        errorX = targetX - self.x
        errorY = targetY - self.y
        errorDist = math.hypot(errorX, errorY)

        errorTheta = math.atan2(math.sin(targetTheta - self.theta), 
                                math.cos(targetTheta - self.theta))

        self.integral_lin += errorDist * self.dt
        self.integral_ang += errorTheta * self.dt

        d_lin = (errorDist - self.prev_err_lin) / self.dt if self.dt > 0 else 0.0
        d_ang = (errorTheta - self.prev_err_ang) / self.dt if self.dt > 0 else 0.0
        self.prev_err_lin, self.prev_err_ang = errorDist, errorTheta

        v = (self.kp_lin * errorDist + self.ki_lin * self.integral_lin + self.kd_lin * d_lin)
        omega = (self.kp_ang * errorTheta + self.ki_ang * self.integral_ang + self.kd_ang * d_ang)

        v = np.clip(v, -20, 20)
        omega = np.clip(omega, -0.2, 0.2)

        heading_to_target = math.atan2(errorY, errorX)
        body_angle = heading_to_target - self.theta

        vx = v * math.cos(body_angle)
        vy = v * math.sin(body_angle)

        if (abs(errorDist) < 2) and (abs(errorTheta) < 0.05):
            vx, vy, omega = 0.0, 0.0, 0.0

        return vx, vy, omega
        
    def inverseKinematic(self, vx, vy, omega):
        L = self.lx + self.ly
        FL = (vx - vy - L * omega) / self.radius 
        FR = (vx + vy + L * omega) / self.radius 
        BL = (vx + vy - L * omega) / self.radius
        BR = (vx - vy + L * omega) / self.radius

        return FL, FR, BL, BR

    def targetToRadPerSec(self, targetX, targetY, targetTheta):
        vx, vy, omega = self.PID(targetX, targetY, targetTheta)
        FL, FR, BL, BR = self.inverseKinematic(vx, vy, omega)

        return FL, FR, BL, BR

    def radPerSecToRPM(self, rad_per_sec):
        rpm = (rad_per_sec*60) / (2*math.pi)
        return rpm
    
    def rpmToPwm(self, rpm, rpm_max, pwm_max):
        pwm = (rpm/rpm_max)*pwm_max
        pwm = np.clip(pwm, -pwm_max, pwm_max)

        return pwm

    def moveToTarget(self, R, L, S, coordinats, pwm_max = 255, rpm_max = 500):
        self.inputEncValue(R, L, S)
        self.pose()

        targetX, targetY, targetTheta = coordinats[self.init_state]
        errorX = targetX - self.x
        errorY = targetY - self.y
        errorDist = math.hypot(errorX, errorY)
        errorTheta = math.atan2(math.sin(targetTheta - self.theta),
                                    math.cos(targetTheta - self.theta))
            
        FL, FR, BL, BR = self.targetToRadPerSec(targetX, targetY, targetTheta)
        RPM_FL = self.radPerSecToRPM(FL)
        RPM_FR = self.radPerSecToRPM(FR)
        RPM_BL = self.radPerSecToRPM(BL)
        RPM_BR = self.radPerSecToRPM(BR)

        PWM_FL = self.rpmToPwm(RPM_FL, rpm_max, pwm_max)
        PWM_FR = self.rpmToPwm(RPM_FR, rpm_max, pwm_max)
        PWM_BL = self.rpmToPwm(RPM_BL, rpm_max, pwm_max)
        PWM_BR = self.rpmToPwm(RPM_BR, rpm_max, pwm_max)

        self.pubPose_.publish(Float32MultiArray(data=[self.x, self.y, (self.theta/(2*math.pi))*360]))
        self.pubFR_.publish(Float32(data=PWM_FR))
        self.pubFL_.publish(Float32(data=PWM_FL))
        self.pubBR_.publish(Float32(data=PWM_BR))
        self.pubBL_.publish(Float32(data=PWM_BL))

        if (errorDist < 2 and abs(errorTheta) < 0.05):
            if self.init_state < (len(coordinats) - 1):
                self.init_state += 1
                self.x = 0.0
                self.y = 0.0
                self.theta = 0.0
                self.integral_lin = 0.0
                self.integral_ang = 0.0
                self.prev_err_lin = 0.0
                self.prev_err_ang = 0.0
            
class Navigation(Node):
    def __init__(self, track_width, forward_offset, radius, lx, ly, 
                 kp_lin = 2.0, ki_lin = 0.0, kd_lin = 1.0, 
                 kp_ang = 2.0, ki_ang = 0.0, kd_ang = 1.0, dt = 0.01):
        
        super().__init__("odom_subs")

        self.robot = Odometri(track_width, forward_offset, radius, lx, ly, 
                              kp_lin, ki_lin, kd_lin, kp_ang, ki_ang, kd_ang, dt)
        self.subs_ = self.create_subscription(Float32MultiArray, 
                                             "odometry", 
                                             self.navigation, 
                                             10)

    def navigation(self, msg):
        self.robot.moveToTarget(R=msg.data[0], L=msg.data[1], S=msg.data[2], coordinats=[(0,0,0)])

def main():
    rclpy.init()
    node = Navigation(track_width=27.25, forward_offset=4.45, radius=13, lx=22, ly=24, dt=0.01)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()
