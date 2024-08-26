#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("hello_world")
        self.get_logger().info("Hello World!")


def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.shutdown()

if __name__=="__main__":
    main()