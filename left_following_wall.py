#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollowing:
    def __init__(self):
        rospy.init_node('wall_following', anonymous=True)

        # Threshold to define whether a wall is "occupied"
        self.distance_threshold = 0.3

        # Ignore any reading closer than this (to avoid floor reflections when tilted)
        self.floor_filter_distance = 0.1

        # Speeds
        self.forward_speed      = 0.15
        self.slow_forward_speed = 0.05
        self.turning_speed      = 0.4

        # ROS subscribers and publishers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.twist = Twist()

    def scan_callback(self, data):
        # Get left (~60° to 90°) and front (~±20°) scan data
        left_ranges  = data.ranges[60:90]
        front_ranges = data.ranges[0:20] + data.ranges[-20:]

        # Filter out infinity readings AND any very close hits (probably floor)
        valid_left = [
            r for r in left_ranges
            if (not math.isinf(r)) and (r > self.floor_filter_distance)
        ]
        valid_front = [
            r for r in front_ranges
            if (not math.isinf(r)) and (r > self.floor_filter_distance)
        ]

        # Determine the minimum distance seen on each side, or +inf if none
        if valid_left:
            min_left = min(valid_left)
        else:
            min_left = float('inf')

        if valid_front:
            min_front = min(valid_front)
        else:
            min_front = float('inf')

        # Determine wall occupancy
        left_occupied  = (min_left < self.distance_threshold)
        front_occupied = (min_front < self.distance_threshold)

        # Decide next move
        self.decide_action(left_occupied, front_occupied)

    def decide_action(self, left_occupied, front_occupied):
        if not left_occupied and not front_occupied:
            # Left = Free, Front = Free → Slowly move forward and curve left
            self.twist.linear.x  = self.slow_forward_speed
            self.twist.angular.z = 0.2
            rospy.loginfo("WALL NOT FOUND - slowly moving forward and curving left")

        elif left_occupied and not front_occupied:
            # Left = Occupied, Front = Free → Go forward
            self.twist.linear.x  = self.forward_speed
            self.twist.angular.z = 0.0
            rospy.loginfo("WALL ON LEFT - moving forward")

        elif not left_occupied and front_occupied:
            # Left = Free, Front = Occupied → Turn right
            self.twist.linear.x  = 0.0
            self.twist.angular.z = -self.turning_speed
            rospy.loginfo("WALL AHEAD - turning right")

        else:  # left_occupied and front_occupied
            # Left = Occupied, Front = Occupied → Turn right
            self.twist.linear.x  = 0.0
            self.twist.angular.z = -self.turning_speed
            rospy.loginfo("WALLS AHEAD AND LEFT - turning right")

        self.cmd_pub.publish(self.twist)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    robot = WallFollowing()
    robot.run()
