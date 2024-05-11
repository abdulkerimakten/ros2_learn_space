#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        # self.get_logger().info("I AM ROS2 all changes worked correctly!")

        self.counter_ = 0

        # using timer
        self.create_timer(1.0, self.timer_callback)

    
    def timer_callback(self):
        self.get_logger().info("HALLO " + str(self.counter_))
        self.counter_ += 1


# --- MAIN PART ---

def main(args=None):
    rclpy.init(args=args)
    # all nodish things are happening here...

    node = MyNode()
    rclpy.spin(node) # keep node alive until kill operation

    rclpy.shutdown()

if __name__== "__main__":
    main()