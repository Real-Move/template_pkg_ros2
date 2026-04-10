#!/usr/bin/env python3

# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node


class ClockNode(Node):
    def __init__(self):
        super().__init__("clock_node")
        self.publisher_ = self.create_publisher(Time, "clock_topic", 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published current time: sec={msg.sec} nanosec={msg.nanosec}")


def main(args=None):
    rclpy.init(args=args)
    node = ClockNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
