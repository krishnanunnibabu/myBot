#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleBoundaryAvoidanceNode(Node):
    def __init__(self):
        super().__init__("turtle_boundary_avoidance")
        self.previous_x_ = 0
        self.previous_y_ = 0
        self.new_theta_ = 0
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Node has started....")

    def pose_callback(self, msg:Pose):
        self.cmd_vel_ = Twist()

        if msg.x < 2.5 or msg.x > 8.5 or msg.y < 2.5 or msg.y > 8.5:
            self.cmd_vel_.linear.x = 1.4
            self.cmd_vel_.angular.z = 1.0
        else:
            self.cmd_vel_.linear.x = 3.0
            self.cmd_vel_.angular.z = 0.0
        self.cmd_vel_pub_.publish(self.cmd_vel_)



def main(args=None):
    rclpy.init(args=args)
    node = TurtleBoundaryAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()