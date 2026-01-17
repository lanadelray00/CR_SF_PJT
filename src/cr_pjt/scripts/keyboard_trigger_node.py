#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import sys
import termios
import tty

### 추후 GUI 버튼 or something 으로 변경
class KeyboardTriggerNode(Node):
    def __init__(self):
        super().__init__('keyboard_trigger_node')
        self.pub = self.create_publisher(Bool, '/pick_and_place/start', 10)
        self.get_logger().info("Keyboard trigger node started")
        self.get_logger().info("Press SPACE to trigger pick & place")
        self.get_logger().info("Press Ctrl+C to exit")

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)

                if ch == '\x03':  # exit : Ctrl+C
                    self.get_logger().info("Ctrl+C detected, exiting")
                    break

                if ch == ' ': # reading space bar input
                    msg = Bool()
                    msg.data = True
                    self.pub.publish(msg)
                    self.get_logger().info("▶ Published /pick_and_place/start = True")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = KeyboardTriggerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard trigger node stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
