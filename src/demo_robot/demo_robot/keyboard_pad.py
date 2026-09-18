#!/usr/bin/python3

import select
import sys
import termios
import threading
import tty

import rclpy
from custom_messages.msg import BetterGamePad
from rclpy.node import Node

class KeyboardPad(Node):
    def __init__(self):
        super().__init__('keyboard_pad_node')

        self.publisher = self.create_publisher(BetterGamePad, '/pad', 10)
        self.lock = threading.Lock()

        self.left_x = 128
        self.left_y = 128
        self.right_x = 128
        self.button_x = False
        self.previous_button_x = False

        self.create_timer(0.05, self.publish_pad)
        threading.Thread(target=self.read_keyboard, daemon=True).start()

        print('Controls: w/s forward/back, a/d left/right, q/e turn')
        print('          k stop moving, x emergency stop (no Enter needed)')

    def read_keyboard(self):
        try:
            terminal = open('/dev/tty')
        except OSError:
            terminal = sys.stdin

        if not terminal.isatty():
            self.get_logger().error('Keyboard input needs an interactive terminal')
            return

        settings = termios.tcgetattr(terminal)
        tty.setcbreak(terminal.fileno())
        try:
            while rclpy.ok():
                readable, _, _ = select.select([terminal], [], [], 0.1)
                if readable:
                    self.apply_key(terminal.read(1).lower())
        finally:
            termios.tcsetattr(terminal, termios.TCSADRAIN, settings)
            if terminal is not sys.stdin:
                terminal.close()

    def apply_key(self, key):
        with self.lock:
            if key == 'w':
                self.left_x = 255
                self.left_y = 128
                self.right_x = 128
            elif key == 's':
                self.left_x = 0
                self.left_y = 128
                self.right_x = 128
            elif key == 'a':
                self.left_x = 128
                self.left_y = 0
                self.right_x = 128
            elif key == 'd':
                self.left_x = 128
                self.left_y = 255
                self.right_x = 128
            elif key == 'q':
                self.left_x = 128
                self.left_y = 128
                self.right_x = 0
            elif key == 'e':
                self.left_x = 128
                self.left_y = 128
                self.right_x = 255
            elif key == 'k':
                self.left_x = 128
                self.left_y = 128
                self.right_x = 128
            elif key == 'x':
                self.button_x = True

    def publish_pad(self):
        with self.lock:
            msg = BetterGamePad()
            msg.left_analog_x = self.left_x
            msg.left_analog_y = self.left_y
            msg.right_analog_x = self.right_x
            msg.right_analog_y = 128
            msg.button_x = self.button_x
            msg.previous_button_x = self.previous_button_x
            self.publisher.publish(msg)

            self.previous_button_x = self.button_x
            self.button_x = False

def main():
    rclpy.init()
    node = KeyboardPad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
