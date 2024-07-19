#!/usr/bin/env python3
'''
    Given generating the data is not related to Pure Pursuit, 
    I copied this file from https://github.dev/artrela/f1tenth/
'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os
import atexit
from scipy.interpolate import splprep, splev

topic = 'ego_racecar/odom'
file = open('./src/pure_pursuit/data/waypoints.csv', 'w')
timer = 0

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        self.subscription = self.create_subscription(Odometry, topic, self.save_waypoint, 10)
        print(f"Saving Waypoints to: wps.csv")
    
    def save_waypoint(self, data):
        global timer
        timer = timer + 1
        if not (timer % 100):
            file.write('%f, %f\n' % (data.pose.pose.position.x, data.pose.pose.position.y))
        
def shutdown():
    file.close()

def main(args=None):
    rclpy.init(args=args)

    waypoint_logger = WaypointLogger()
    atexit.register(shutdown)
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()