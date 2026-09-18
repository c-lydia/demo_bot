#!/usr/bin/python3

import rclpy
from custom_messages.msg import BetterGamePad
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyPad(Node):
    def __init__(self):
        super().__init__('joy_pad_node')
        self.pad_pub = self.create_publisher(BetterGamePad, '/pad', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.previous_button_x = False

    @staticmethod
    def analog_value(value):
        value = max(-1.0, min(1.0, value))
        return int(round(128 + 127 * value))

    @staticmethod
    def axis(msg, index):
        if index >= len(msg.axes):
            return 0.0
        return msg.axes[index]

    @staticmethod
    def button(msg, index):
        if index >= len(msg.buttons):
            return False
        return bool(msg.buttons[index])

    def joy_cb(self, msg: Joy):
        button_x = self.button(msg, 2)

        pad = BetterGamePad()
        pad.left_analog_x = self.analog_value(self.axis(msg, 1))
        pad.left_analog_y = self.analog_value(self.axis(msg, 0))
        pad.right_analog_x = self.analog_value(self.axis(msg, 3))
        pad.right_analog_y = self.analog_value(self.axis(msg, 4))
        pad.button_x = button_x
        pad.previous_button_x = self.previous_button_x
        self.pad_pub.publish(pad)

        self.previous_button_x = button_x

def main():
    rclpy.init()
    node = JoyPad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
