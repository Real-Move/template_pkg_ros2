#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2024 Real-Move
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Time


class PrintClockNode(Node):
    def __init__(self):
        super().__init__('print_clock_node')
        self.publisher_ = self.create_publisher(Time, 'clock_topic', 10)
    #     self.timer_ = self.create_timer(1.0, self.timer_callback)

    # def timer_callback(self):
    #     msg = Time()
    #     msg.data = self.get_clock().now().to_msg()
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f"Published current time: {msg.data}")

def main(args=None):
    rclpy.init(args=args)

    # node = PrintClockNode()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass

    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
