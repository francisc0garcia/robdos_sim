#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import math
import numpy as np
from mavros_msgs.msg import WaypointList, Waypoint, RCOut
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class PositionController:
    def __init__(self, current_target):
        self.current_target = current_target

        # define rate of  10 hz
        self.rate = rospy.Rate(50.0)

        # init variables for odometry
        [self.robot_position_x, self.robot_position_y, self.robot_position_z] = [0, 0, 0]
        [self.robot_roll, self.robot_pitch, self.robot_yaw] = [0, 0, 0]

        self.is_positioned = False
        self.is_oriented = True

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("/mavros/rc/out", RCOut, queue_size=1)
        self.RCOut_msg = RCOut()

        ######################################################SUBSCRIBERS########################################################
        # create subscriber for robot localization
        # self.sub_localization = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.process_localization_message, queue_size=1)
        self.sub_localization = rospy.Subscriber('/robdos/odom', Odometry, self.process_localization_message,
                                                 queue_size=1)


    def init_controller(self):
        ######################################################CONTROLLER########################################################
        # if not reach goal and correct orientation
        while not rospy.is_shutdown() and not self.is_positioned and self.is_oriented:
            self.update_controller()
            self.rate.sleep()

        return [self.is_oriented, self.is_positioned]

        # update position and orientation of robot (odometry)
    def process_localization_message(self, odometry_msg):
        self.robot_position_x = odometry_msg.pose.pose.position.x
        self.robot_position_y = odometry_msg.pose.pose.position.y
        self.robot_position_z = odometry_msg.pose.pose.position.z

        quaternion = (
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w)

        (self.robot_roll, self.robot_pitch, self.robot_yaw) = euler_from_quaternion(quaternion)

        self.update_controller()

    def update_controller(self):
        kp = 500
        K_output = 0.0
        distance_x = self.current_target[0] - self.robot_position_x
        distance_y = self.current_target[1] - self.robot_position_y

        # update controller
        desired_yaw = math.atan2(self.current_target[0], self.current_target[1])

        yaw_error =  desired_yaw - self.robot_yaw
        error = math.sqrt(math.pow(distance_x, 2.0) + math.pow(distance_y, 2.0))

        # if robot is oriented
        if abs(yaw_error) < 0.1 :
            self.is_oriented = True

            # if robot do not reach jet desired position
            if error > 1:
                self.is_positioned = False

                K_output = 1100
                K_output = self.bound_limit(K_output, 1100, 1900)

                # set speed thrusters
                self.RCOut_msg = RCOut()
                self.RCOut_msg.header.stamp = rospy.Time.now()
                self.RCOut_msg.header.frame_id = "0"
                self.RCOut_msg.channels = [K_output, K_output, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
                self.mavros_vel_pub.publish(self.RCOut_msg)
            else:
                # Robot reach goal and it is oriented

                # stop motors
                self.RCOut_msg = RCOut()
                self.RCOut_msg.header.stamp = rospy.Time.now()
                self.RCOut_msg.header.frame_id = "0"
                self.RCOut_msg.channels = [1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
                self.mavros_vel_pub.publish(self.RCOut_msg)

                self.is_positioned = True
        else:
            # Fix orientation
            self.is_oriented = False
            self.is_positioned = False


    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)
