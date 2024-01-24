# Copyright (c) 2024 Luca Fortini

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
