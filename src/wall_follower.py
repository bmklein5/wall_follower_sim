#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header


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
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)


    def callback(self, data):
        drive_stamp = AckermannDriveStamped()
        drive_stamp.header.stamp=rospy.Time.now()
        drive_stamp.header.frame_id="wall_follower" # or is frame_id="base_link"??
        drive_stamp.drive.steering_angle = .1
        drive_stamp.drive.steering_angle_velocity = 1
        drive_stamp.drive.speed = self.VELOCITY
        drive_stamp.drive.acceleration = 0
        drive_stamp.drive.jerk = 0

        self.pub.publish(drive_stamp)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
