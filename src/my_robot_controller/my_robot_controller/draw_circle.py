#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawCircleNode(Node):

    def __init__(self):
        super().__init__("draw_circle")

        # creating a publisher
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # call the function in every 0.5 second.
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)

        self.get_logger().info("Draw circle node has been started and drawing circle.")


    def send_velocity_command(self):
        # creating message
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        # send message
        self.cmd_vel_pub_.publish(msg=msg)


# --- MAIN PART ---

def main(args=None):
    rclpy.init(args=args)

    node = DrawCircleNode()
    rclpy.spin(node=node)

    rclpy.shutdown()