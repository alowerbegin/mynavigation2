#!/usr/bin/env python3
"""Send only the selected chassis command to the simulated drive."""

import math
import time

import rclpy
from byd_custom_msgs.msg import ControlRes
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ControlResToTwist(Node):
    def __init__(self):
        super().__init__('control_res_to_twist')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            ControlRes, '/control_to_uart', self.on_command, 1)
        self.last_command = None
        self.timer = self.create_timer(0.05, self.check_timeout)

    def on_command(self, message):
        if self.count_publishers('/control_to_uart') != 1:
            self.last_command = None
            self.publisher.publish(Twist())
            return
        if not (math.isfinite(message.v) and math.isfinite(message.w)):
            self.last_command = None
            self.publisher.publish(Twist())
            return
        command = Twist()
        command.linear.x = message.v
        command.angular.z = message.w
        self.last_command = time.monotonic()
        self.publisher.publish(command)

    def check_timeout(self):
        if self.last_command is None or time.monotonic() - self.last_command > 0.2:
            self.publisher.publish(Twist())


def main():
    rclpy.init()
    node = ControlResToTwist()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
