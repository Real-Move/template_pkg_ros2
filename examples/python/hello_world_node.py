#!/usr/bin/env python3

# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node("hello_world_node")
    node.get_logger().info("Hello, world!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
