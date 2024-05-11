#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
# When you add any additional libraries,  don't forget to add them in dependencies in XML

class PoseSubscriberNode(Node):
    
    def __init__(self):
        super().__init__("pose_subsriber")

        # creating subscriber
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        self.get_logger().info("Pose Subscriber has been created")

    def pose_callback(self, msg: Pose):
        # show the message info that is gotten by the subscriber from the topic
        self.get_logger().info("(x: " + str(msg.x) + ", " + "y: " + str(msg.y) + ")")

def main(args=None):
    rclpy.init(args=args)

    node = PoseSubscriberNode()
    rclpy.spin(node=node)

    rclpy.shutdown()