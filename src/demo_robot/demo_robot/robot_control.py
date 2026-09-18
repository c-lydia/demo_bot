#!/usr/bin/python3

import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_messages.msg import BetterGamePad
from sensor_msgs.msg import Imu

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.gamepad_sub = self.create_subscription(BetterGamePad, '/pad', self.gamepad_cb, 10)

        self.stop = False

        self.gamepad_vx = 0.0
        self.gamepad_vy = 0.0
        self.gamepad_wz = 0.0

        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0
        self.kp = 1.0

        self.yaw_start = None
        self.previous_yaw_raw = 0.0
        self.overflow_counter = 0.0
        self.current_yaw = 0.0
        self.desired_yaw = 0.0
        self.prev_time = None

        self.timer = self.create_timer(0.01, self.timer_cb)

    def imu_cb(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y 
        qz = msg.orientation.z 
        qw = msg.orientation.w 
        
        yaw_raw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy ** 2.0 + qz ** 2.0))
        
        if self.yaw_start is None:
            self.yaw_start = yaw_raw 
            self.previous_yaw_raw = yaw_raw
            
        difference_yaw = yaw_raw - self.previous_yaw_raw
        
        if abs(difference_yaw) > math.pi:
            if difference_yaw > 0.0:
                self.overflow_counter -= 1.0
            else:
                self.overflow_counter += 1.0
                
        self.previous_yaw_raw = yaw_raw

        self.current_yaw = yaw_raw - self.yaw_start + 2.0 * math.pi * self.overflow_counter

    def gamepad_cb(self, msg: BetterGamePad):
        if msg.button_x and not msg.previous_button_x:
            self.stop = True

        self.gamepad_vx = (msg.left_analog_x - 128)/128
        self.gamepad_vy = (msg.left_analog_y - 128)/128
        self.gamepad_wz = (msg.right_analog_x - 128)/128

        if abs(self.gamepad_vx) <= 0.1:
            self.gamepad_vx = 0.0
        if abs(self.gamepad_vy) <= 0.1:
            self.gamepad_vy = 0.0
        if abs(self.gamepad_wz) <= 0.1:
            self.gamepad_wz = 0.0

        if abs(self.gamepad_vx) > self.max_linear_speed:
            self.gamepad_vx = self.sign(self.gamepad_vx) * self.max_linear_speed
        if abs(self.gamepad_vy) > self.max_linear_speed:
            self.gamepad_vy = self.sign(self.gamepad_vy) * self.max_linear_speed
        if abs(self.gamepad_wz) > self.max_angular_speed:
            self.gamepad_wz = self.sign(self.gamepad_wz) * self.max_angular_speed

    def timer_cb(self):
        current_time = time.perf_counter()

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        self.desired_yaw += self.gamepad_wz * dt
        error_yaw = self.desired_yaw - self.current_yaw
        wz = self.kp * error_yaw

        if abs(wz) > self.max_angular_speed: 
            wz = self.sign(wz) * self.max_angular_speed

        if not self.stop:
            self.publish_cmd(self.gamepad_vx, self.gamepad_vy, wz)
        else:
            self.publish_stop(0.0, 0.0, 0.0)

    def publish_cmd(self, vx, vy, wz):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vx
        cmd_vel_msg.linear.y = vy
        cmd_vel_msg.angular.z = wz
        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.get_logger().info(f'Publishing: vx = {cmd_vel_msg.linear.x}, vy = {cmd_vel_msg.linear.y}, wz = {cmd_vel_msg.angular.z}')

    def publish_stop(self, vx, vy, wz):
        self.publish_cmd(vx, vy, wz)

    def sign(self, x):
        if x < 0.0:
            return -1.0
        else:
            return 1.0

def main():
    rclpy.init()
    robot_control_node = RobotControl()
    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
