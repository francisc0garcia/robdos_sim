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

class OrientationController:
    def __init__(self, current_target):
        self.current_target = current_target

        # define rate of  10 hz
        self.rate = rospy.Rate(50.0)

        # init variables for odometry
        [self.robot_position_x, self.robot_position_y, self.robot_position_z] = [0, 0, 0]
        [self.robot_roll, self.robot_pitch, self.robot_yaw] = [0, 0, 0]

        self.is_oriented = False

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("/mavros/rc/out", RCOut, queue_size=1)
        self.RCOut_msg = RCOut()

        ######################################################SUBSCRIBERS########################################################
        # create subscriber for robot localization
        # self.sub_localization = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.process_localization_message, queue_size=1)
        self.sub_localization = rospy.Subscriber('/robdos/odom', Odometry, self.process_localization_message,
                                                 queue_size=1)

        ######################################################CONTROLLER########################################################
        # infinity loop
        while not rospy.is_shutdown() and not self.is_oriented:
            self.update_controller()
            self.rate.sleep()


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

        # update controller
        desired_yaw = math.atan2(self.current_target[0] - self.robot_position_x, self.current_target[1]- self.robot_position_y)

        yaw_error = desired_yaw - self.robot_yaw

        kp = 300

        if abs(yaw_error) > 1.85:
            self.is_oriented = False
            K_output = -kp * yaw_error

            K_LM = 1500 + K_output
            K_RM = 1500 - K_output

            K_LM = self.bound_limit(K_LM, 1100, 1900)
            K_RM = self.bound_limit(K_RM, 1100, 1900)

            # set speed thrusters
            self.RCOut_msg = RCOut()
            self.RCOut_msg.header.stamp = rospy.Time.now()
            self.RCOut_msg.header.frame_id = "0"
            self.RCOut_msg.channels = [K_LM, K_RM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.mavros_vel_pub.publish(self.RCOut_msg)

        else:
            K_LM = 1500
            K_RM = 1500

            # set speed thrusters
            self.RCOut_msg = RCOut()
            self.RCOut_msg.header.stamp = rospy.Time.now()
            self.RCOut_msg.header.frame_id = "0"
            self.RCOut_msg.channels = [K_LM, K_RM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            self.mavros_vel_pub.publish(self.RCOut_msg)

            self.is_oriented = True


    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)
