#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
from math import sqrt, atan2
import time

class TurtleCatchNode(Node):
    def __init__(self):
        super().__init__("catcher")
        self.get_logger().info("Node has started")
        self.pose = Pose()
        self.threshold = 0.5
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", callback=self.get_pose, qos_profile=10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", qos_profile=10)
        self.create_timer(0.1, self.main)
        self.spawn()
        

    def main(self):
        self.distance = self.eucheuclidean_distance_calc()
        self.azimuth = self.azimuth_calc()
        self.turtle_control()
        
        if self.distance < self.threshold:
            self.kill()
            time.sleep(0.1)
            self.spawn()

    

    def linear_vel(self, const=1):
        return const * self.distance
    
    def angular_vel(self, const=1.0):
        return const * (self.azimuth - self.pose.theta)

    def turtle_control(self):
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = self.linear_vel(2.0)
        self.cmd_vel.angular.z = self.angular_vel(8.0)
        self.cmd_vel_pub.publish(self.cmd_vel)

    def get_pose(self, data: Pose):
        self.pose = data

        
        
    def eucheuclidean_distance_calc(self):
        return sqrt((self.pose.x - self.x)**2 + (self.pose.y - self.y)**2)

    def azimuth_calc(self):
        return atan2((self.y - self.pose.y), (self.x - self.pose.x))

    def spawn(self):
        self.x = random.uniform(0,11)
        self.y = random.uniform(0,11)
        self.theta = random.uniform(0,6.28)
        self.name = 'new_turtle'
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(1):
            self.get_logger().info("Waiting for spawn service")
        
        request = Spawn.Request()
        request.x = self.x
        request.y = self.y
        request.theta = self.theta
        request.name = self.name
        future = client.call_async(request)
        

    def kill(self):
        client = self.create_client(Kill, "/kill")  
        request = Kill.Request()
        request.name = "new_turtle"
        future = client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    catcher = TurtleCatchNode()
    try:
        rclpy.spin(catcher)
    except KeyboardInterrupt:
        pass
    
    catcher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
