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
from dynamic_reconfigure.server import Server
from robdos_sim.cfg import controllerOrientationConfig

from PID import *

class OrientationController:
    def __init__(self, current_target):
        self.degrees2rad = math.pi/180.0
        self.rad2degrees = 180.0/math.pi

        self.current_target = current_target
        self.list_points = []

        # define rate of  10 hz
        self.rate = rospy.Rate(50.0)

        # init variables for odometry
        [self.robot_position_x, self.robot_position_y, self.robot_position_z] = [0, 0, 0]
        [self.robot_roll, self.robot_pitch, self.robot_yaw] = [0, 0, 0]

        self.is_oriented = False

        """initialize controllers"""
        self.threshold_orientation = 0.0

        self.S_kp = 0.0
        self.S_kd = 0.0
        self.S_ki = 0.0
        self.S_windup = 0.0

        self.controller_pid =  PID()
        self.controller_pid.SetPoint = 0.0
        self.controller_pid.setSampleTime(0.01)
        self.controller_pid.setKp(self.S_kp)
        self.controller_pid.setKd(self.S_kd)
        self.controller_pid.setKi(self.S_ki)
        self.controller_pid.setWindup(self.S_windup)

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("/mavros/rc/out", RCOut, queue_size=1)
        self.RCOut_msg = RCOut()

        ######################################################SUBSCRIBERS########################################################
        # create subscriber for robot localization
        # self.sub_localization = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.process_localization_message, queue_size=1)
        self.sub_localization = rospy.Subscriber('/robdos/odom', Odometry, self.process_localization_message,
                                                 queue_size=1)

        self.sub_waypoint_list = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.process_waypoint_message, queue_size=1)

        # reconfigure service
        self.srv = Server(controllerOrientationConfig, self.reconfig_callback) # define dynamic_reconfigure callback

        ######################################################CONTROLLER########################################################
        # infinity loop
        while not rospy.is_shutdown() and not self.is_oriented:
            # only update when receive new odometry message
            #self.update_controller()
            self.rate.sleep()

        self.srv.set_service.shutdown("close process")

    # update list of waypoints
    def process_waypoint_message(self, waypoint_list_msg):
        self.list_points = []

        for waypoint in waypoint_list_msg.waypoints:
            # set last value to False: means not reached jet.
            self.list_points.append( [waypoint.x_lat, waypoint.y_long, waypoint.z_alt, waypoint.is_current] )

        if len(self.list_points) > 0:
            point = self.list_points[1]
            self.current_target = [point[0], point[1], point[2] ]

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
        desired_yaw = math.atan2(self.current_target[1] - self.robot_position_y, self.current_target[0] - self.robot_position_x )

        # update controller (using degrees)
        self.controller_pid.SetPoint = desired_yaw
        self.controller_pid.update( self.robot_yaw )
        self.controller_pid.output = self.bound_limit(self.controller_pid.output , -1000, 1000)

        if abs(self.controller_pid.error) > (self.threshold_orientation * self.degrees2rad):
            self.is_oriented = False
            #K_output = -kp * yaw_error
            K_output = self.controller_pid.output
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

    def reconfig_callback(self, config, level):
        self.S_kp = config['ori_kp']
        self.S_kd = config['ori_kd']
        self.S_ki = config['ori_ki']
        self.S_windup = config['ori_wu']

        self.threshold_orientation = config['thr_orientation']

        self.controller_pid.setKp(self.S_kp)
        self.controller_pid.setKd(self.S_kd)
        self.controller_pid.setKi(self.S_ki)
        self.controller_pid.setWindup(self.S_windup)
        self.controller_pid.error = 0.0
        self.controller_pid.output = 0.0

        return config

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)
