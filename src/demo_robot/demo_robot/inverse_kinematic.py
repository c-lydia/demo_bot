#!/usr/bin/python3

import rclpy
import time
from rclpy.node import Node
from custom_messages.msg import MotorCommand
from geometry_msgs.msg import Twist

A_MAX = 15.0

class InverseKinematic(Node):
    def __init__(self):
        super().__init__('inverse_kinematic_node')
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.kinematic_pub = self.create_publisher(MotorCommand, '/publish_motor', 10)

        self.lx = 20.0
        self.ly = 17.0
        self.r = 0.06

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.prev_time = None
        self.prev_motor_speed = [None, None, None, None]

        self.timer = self.create_timer(0.01, self.timer_callback)

    def cmd_vel_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def process_cmd(self, vx, vy, wz):
        wheel = [
            vx/self.r - vy/self.r - wz * (self.lx + self.ly)/(2.0 * self.r),
            vx/self.r + vy/self.r + wz * (self.lx + self.ly)/(2.0 * self.r),
            vx/self.r - vy/self.r + wz * (self.lx + self.ly)/(2.0 * self.r),
            vx/self.r + vy/self.r - wz * (self.lx + self.ly)/(2.0 * self.r)
        ]
        return wheel

    def timer_callback(self):
        current_time = time.perf_counter()

        if self.prev_time is None:
            self.prev_time = current_time

        dt = current_time - self.prev_time

        if self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0:
            motor_speed = [0.0, 0.0, 0.0, 0.0]
        else:
            motor_speed = self.process_cmd(self.vx, self.vy, self.wz)
            for i in range(4):
                motor_speed[i] = self.rate_limit(
                    motor_speed[i],
                    A_MAX,
                    self.prev_motor_speed[i],
                    dt,
                )

        for i in range(4):
            motor_msg = MotorCommand()
            motor_msg.can_id = i + 1
            motor_msg.goal = motor_speed[i]
            motor_msg.speedmode = True
            self.kinematic_pub.publish(motor_msg)
            self.get_logger().info(f'Publishing: {motor_msg}')

            self.prev_motor_speed[i] = motor_speed[i]

        self.prev_time = current_time

    def rate_limit(self, v, a_max, v_prev, dt):
        if v_prev is None:
            return v

        dv_max = a_max * dt
        dv = v - v_prev

        if abs(dv) > dv_max:
            if dv > 0:
                dv = dv_max
            else:
                dv = -dv_max

        return dv + v_prev

def main():
    rclpy.init()
    inverse_kinematic_node = InverseKinematic()
    try:
        rclpy.spin(inverse_kinematic_node)
    except KeyboardInterrupt:
        pass
    finally:
        inverse_kinematic_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
