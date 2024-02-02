#!/usr/bin/env python3

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
