#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
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
        self.set_pen(183, 169, 107)
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

    def turtle_control(self):
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = self.linear_vel(2.0)
        self.cmd_vel.angular.z = self.angular_vel(8.0)
        self.cmd_vel_pub.publish(self.cmd_vel)

    def get_pose(self, data: Pose):
        self.pose = data

    def spawn(self):
        self.set_pen(183, 169, 107)
        self.x = random.uniform(0,11)
        self.y = random.uniform(0,11)
        while abs(self.y - self.pose.y) < 3:
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
        client.call_async(request)
        
        

    def kill(self):
        client = self.create_client(Kill, "/kill")  
        request = Kill.Request()
        request.name = "new_turtle"
        future = client.call_async(request)
        future.add_done_callback(self.kill_callback)

    def kill_callback(self, future):
        try:
            future.result()   
            self.get_logger().info("Turtle caught")
        except Exception as e:
            self.get_logger().warn(f"{e}")

    def set_pen(self, r:int, g:int, b:int, width:int = 4, off:int = 0):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        client.call_async(request)



    def eucheuclidean_distance_calc(self):
        return sqrt((self.pose.x - self.x)**2 + (self.pose.y - self.y)**2)

    def azimuth_calc(self):
        return atan2((self.y - self.pose.y), (self.x - self.pose.x))
    
    def linear_vel(self, const=1):
        return const * self.distance
    
    def angular_vel(self, const=1.0):
        return const * (self.azimuth - self.pose.theta)


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
