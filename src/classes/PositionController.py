#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import math
import numpy as np
from mavros_msgs.msg import WaypointList, Waypoint, OverrideRCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from robdos_sim.msg import StateEvent

from PID import *

class PositionController:
    def __init__(self, current_target):
        self.degrees2rad = math.pi/180.0
        self.rad2degrees = 180.0/math.pi

        self.current_target = current_target

        # define rate of  100 hz
        self.rate = rospy.Rate(50.0)

        # init variables for odometry
        [self.robot_position_x, self.robot_position_y, self.robot_position_z] = [0, 0, 0]
        [self.robot_roll, self.robot_pitch, self.robot_yaw] = [0, 0, 0]

        self.is_positioned = False
        self.is_oriented = True

        """initialize controller"""
        self.threshold_position = 0.0
        self.threshold_orientation = 100 # degree of tolerance

        self.S_kp = 1000.0
        self.S_kd = 100.0
        self.S_ki = 0.0
        self.S_windup = 0.0

        self.controller_pid =  PID()
        self.controller_pid.SetPoint = 0.0
        self.controller_pid.setSampleTime(0.01)
        self.controller_pid.setKp(self.S_kp)
        self.controller_pid.setKd(self.S_kd)
        self.controller_pid.setKi(self.S_ki)
        self.controller_pid.setWindup(self.S_windup)

        # event publisher
        self.event_pub = rospy.Publisher("/robdos/stateEvents", StateEvent, queue_size=1)
        self.event_msg = StateEvent()

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.RCOR_msg = OverrideRCIn()

        ######################################################SUBSCRIBERS########################################################
        # create subscriber for robot localization
        # self.sub_localization = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.process_localization_message, queue_size=1)
        self.sub_localization = rospy.Subscriber('/robdos/odom', Odometry, self.process_localization_message, queue_size=1)
        #self.sub_localization = rospy.Subscriber('/mavros/global_position/local', Odometry, self.process_localization_message, queue_size=1)

        self.sub_waypoint_list = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.process_waypoint_message, queue_size=1)
    
        # update list of waypoints
    def process_waypoint_message(self, waypoint_list_msg):
        self.list_points = []

        for waypoint in waypoint_list_msg.waypoints:
            # set last value to False: means not reached jet.
            self.list_points.append( [waypoint.x_lat, waypoint.y_long, waypoint.z_alt, waypoint.is_current] )

        if len(self.list_points) > 0:
            point = self.list_points[1]
            self.current_target = [point[0], point[1], point[2] ]
             
    def register(self):
        self.sub_localization = rospy.Subscriber('/robdos/odom', Odometry, self.process_localization_message, queue_size=1)
        #self.sub_localization = rospy.Subscriber('/mavros/global_position/local', Odometry, self.process_localization_message, queue_size=1) 
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
        rospy.logerr("position controller")
        distance_x = self.current_target[0] - self.robot_position_x
        distance_y = self.current_target[1] - self.robot_position_y

        self.controller_pid.SetPoint = 0.0

        # control objective: reduce squared distance to goal position
        self.controller_pid.update(math.sqrt(math.pow(distance_x, 2.0) + math.pow(distance_y, 2.0)))

        # estimate correct orientation
        desired_yaw = math.atan2(self.current_target[1]- self.robot_position_y, self.current_target[0] - self.robot_position_x )
        error_orientation = desired_yaw - self.robot_yaw

        # compute error in degrees
        error_orientation = (error_orientation % 2*math.pi)*self.rad2degrees

        # if robot is oriented
        if abs(error_orientation) <= self.threshold_orientation :

            K_output = 1900 #1500 + self.controller_pid.output
            K_output = self.bound_limit(K_output, 1100, 1900)

            # set speed thrusters
            self.RCOR_msg = OverrideRCIn()
            self.RCOR_msg.channels = [0, 0, 0, 1500, 0, K_output, 0, 0]
            self.mavros_vel_pub.publish(self.RCOR_msg)

        else:
            rospy.logerr("not oriented")
            msg_ = StateEvent()
            self.event_msg.cmd = msg_.NOT_ORIENTED
            self.event_pub.publish(self.event_msg)
            self.sub_localization.unregister()

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)
