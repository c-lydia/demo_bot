#!/usr/bin/python3

import rclpy
from custom_messages.msg import MotorCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node

class SimMotorAdapter(Node):
    def __init__(self):
        super().__init__('sim_motor_adapter_node')

        self.lx = 20.0
        self.ly = 17.0
        self.r = 0.06

        self.wheel_speed = [0.0, 0.0, 0.0, 0.0]
        self.updated_motors = set()

        self.motor_sub = self.create_subscription(MotorCommand, '/publish_motor', self.motor_cb, 10)
        self.sim_cmd_pub = self.create_publisher(Twist, '/sim_cmd_vel', 10)

    def motor_cb(self, msg: MotorCommand):
        if msg.can_id < 1 or msg.can_id > 4:
            return

        index = msg.can_id - 1

        if msg.speedmode:
            self.wheel_speed[index] = msg.goal
        else:
            return

        self.updated_motors.add(index)

        if len(self.updated_motors) == 4:
            self.publish_sim_command()
            self.updated_motors.clear()

    def publish_sim_command(self):
        w1, w2, w3, w4 = self.wheel_speed
        rotation_radius = (self.lx + self.ly)/2.0

        msg = Twist()
        msg.linear.x = self.r * (w1 + w2 + w3 + w4)/4.0
        msg.linear.y = self.r * (-w1 + w2 - w3 + w4)/4.0
        msg.angular.z = (self.r * (-w1 + w2 + w3 - w4)/(4.0 * rotation_radius))
        self.sim_cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = SimMotorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
