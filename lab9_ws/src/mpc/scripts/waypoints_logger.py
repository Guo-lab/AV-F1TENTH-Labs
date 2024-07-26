#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np
import csv, tf_transformations
import atexit


file = open('./src/mpc/data/waypoints.csv', 'w')
timer = 0


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger_node')

        odom_topic = "ego_racecar/odom"

        # subscribe to odometry topic
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0

        # Timer to publish messages as fast as possible
        self.timer = self.create_timer(0.11, self.timer_callback)

    def odom_callback(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, 
             odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(q)
        self.yaw = yaw
        self.v = np.sqrt(odom_msg.twist.twist.linear.x**2 + odom_msg.twist.twist.linear.y**2 + odom_msg.twist.twist.linear.z**2)
    
    def timer_callback(self):
        file.write('%f, %f, %f, %f\n' % (self.x, self.y, self.yaw, self.v))
        

def shutdown():
    file.close()
        
def main(args=None):
    rclpy.init(args=args)
    print("Record Waypoints Initialized")
    waypoint_logger = WaypointLogger()
    atexit.register(shutdown)
    
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()


