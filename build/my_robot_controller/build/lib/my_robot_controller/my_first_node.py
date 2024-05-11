#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello from ROS2!!!")

def main(args=None):
    rclpy.init(args=args)
    # all nodish things are happening here...

    node = MyNode()
    rclpy.spin(node) # keep node alive until kill operation

    rclpy.shutdown()

if __name__== "__main__":
    main()