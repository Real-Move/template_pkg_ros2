#!/usr/bin/env python3

# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import rclpy
from rclpy.node import Node


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create a ROS 2 node
    node = Node("hello_world_node")

    # Broadcast a simple log message
    node.get_logger().info("Hello, world!")

    # Process ROS 2 callbacks until receiving a SIGINT (Ctrl+C)
    rclpy.spin(node)

    # Shutdown ROS 2
    rclpy.shutdown()


if __name__ == "__main__":
    # Entry point for the script
    main()
