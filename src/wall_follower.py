#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
from visualization_tools import *
from copy import deepcopy

class InvalidSideException(Exception):
    pass

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # Initialize your publishers and subscribers here
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.callback)

        self.viz = rospy.Publisher("/wall", Marker, queue_size=1)

    def slice_scan(self, scan, angles):
        # print(scan.ranges.size, len(angles))
        i = scan.ranges.size / 2
        # print(i)
        # MAYBE: cast sizes as int
        if self.SIDE == 1:
            scan.ranges = scan.ranges[:i]
            angles = angles[:i]
            # scan.ranges = scan.ranges[:int(scan.ranges.size) / 2]
            # angles = angles[:int(scan.ranges.size) / 2]
        elif self.SIDE == -1:
            scan.ranges = scan.ranges[i:]
            angles = angles[i:]
            # scan.ranges = scan.ranges[int(scan.ranges.size) / 2:]
            # angles = angles[int(scan.ranges.size) / 2:]
        else:
            raise InvalidSideException
        # print(scan.ranges.size, len(angles))
        return scan, angles

    def scan_angles(self, scan):
        inc = scan.angle_increment
        angles = np.array([scan.angle_min])
        for i in range(1, scan.ranges.size):
            angles = np.append(angles, angles[-1] + inc)
        return angles

    def find_wall(self, ranges, angles):
        x = np.cos(angles) * ranges
        y = np.sin(angles) * ranges

        # dist_to_wall = np.mean(np.sqrt(x_squared + y_squared))
        regression = np.polyfit(x, y, 1)
        # [theta, dist_to_wall] = np.polyfit(x, y, 1)
        # return theta, dist_to_wall
        return regression

    def controller(self, theta, dist_to_wall):
        """PD Controller"""
        K_p = 5
        K_d = 1
        error = self.DESIRED_DISTANCE - dist_to_wall
        action = K_p * error + K_d * theta * self.VELOCITY
        return action


    def callback(self, data):
        angles = self.scan_angles(data)
        new_scan = deepcopy(data)
        sliced_scan, sliced_angles = self.slice_scan(new_scan, angles)  # get scan data relevant to side we want

        wall_regression = self.find_wall(sliced_scan.ranges, sliced_angles)
        theta = wall_regression[0]
        dist_to_wall = wall_regression[1]
        action = self.controller(theta, dist_to_wall)

        drive = AckermannDriveStamped()
        drive.header.stamp=rospy.Time.now()
        drive.header.frame_id="wall_follower"  # or is frame_id="base_link"??
        drive.drive.steering_angle = action
        drive.drive.steering_angle_velocity = 1
        drive.drive.speed = self.VELOCITY
        # drive.drive.speed = 4
        drive.drive.acceleration = 0
        drive.drive.jerk = 0

        self.pub.publish(drive)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
