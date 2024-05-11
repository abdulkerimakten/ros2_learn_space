#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial


class TurtleControllerNode(Node):
    """
    we create both subscriber and publisher node in one file and make our turtle robot just move around inside the window.
    we didn't want the robot hit the wall so we include necessary conditions.
    """

    def __init__(self):
        super().__init__("turtle_contorller")

        # exposition to track
        self.previous_x_ = 0

        # publisher
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # subscriber
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        self.get_logger().info("Turtle Controller has been started.")

    # operations that are based on Position informations
    def pose_callback(self, pose: Pose):
        cmd = Twist()

        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            # we focus on going linear 5.0
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        # let's publish this cmd
        self.cmd_vel_publisher_.publish(cmd)

        ## Changing the color of pen

        if pose.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to RED")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x < 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to GREEN")
            self.call_set_pen_service(0, 255, 0, 3, 0)
    

    # we will create a client for the service that will change the color of pen
    def call_set_pen_service(self, r, g, b, width, off):

        client = self.create_client(SetPen, "/turtle1/set_pen")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
            ## if the service is not up after 1 second thise warning occurs

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off



        future = client.call_async(request=request)
        future.add_done_callback(partial(self.callback_set_pen))
        """
        future.add_done_callback(partial(self.callback_set_pen)) line adds a callback function to a future object. 
        This callback function is automatically invoked when the future object is completed
        """

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service has failed: %r" %(e,))

# --MAIN--

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node=node)
    rclpy.shutdown()