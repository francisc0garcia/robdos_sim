#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import numpy as np
from mavros_msgs.msg import WaypointList, Waypoint, RCOut
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class robdos_waypoint_controller:
    def __init__(self):

        # define rate of  10 hz
        self.rate = rospy.Rate(50.0)

        # init variables for odometry
        [self.robot_position_x, self.robot_position_y, self.robot_orientation,
         self.robot_roll, self.robot_pitch, self.robot_yaw] = [0, 0, 0, 0, 0, 0]

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("/mavros/rc/out",  RCOut, queue_size=1)
        self.RCOut_msg = RCOut()

        # create subscriber for robot odometry
        self.sub_odometry = rospy.Subscriber('/robdos/odom', Odometry, self.process_odometry_message, queue_size=1)

        # create subscriber for mavros/waypointlist
        self.sub_waypoint_list = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.process_waypoint_message, queue_size=1)
        self.list_points = []
        self.is_waypoint_loaded = False
        self.current_target = [0, 0, 0]

        # infinity loop
        while not rospy.is_shutdown():
            self.update_controller()
            self.rate.sleep()

    # update position and orientation of robot (odometry)
    def process_odometry_message(self, odometry_msg):
        self.robot_position_x = odometry_msg.pose.pose.position.x
        self.robot_position_y = odometry_msg.pose.pose.position.y
        self.robot_orientation = odometry_msg.pose.pose.orientation.y

        quaternion = (
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w)

        (self.robot_roll, self.robot_pitch, self.robot_yaw) = euler_from_quaternion(quaternion)

        self.update_controller()

    # update list of waypoints
    def process_waypoint_message(self, waypoint_list_msg):
        self.list_points = []

        for waypoint in waypoint_list_msg.waypoints:
            # set last value to False: means not reached jet.
            self.list_points.append( [waypoint.x_lat, waypoint.y_long, waypoint.z_alt, waypoint.is_current] )

    def update_controller(self):
        if len(self.list_points) > 0:
            point = self.list_points[1]
            self.current_target = [point[0], point[1], point[2] ]

        # update controller
        ref = self.current_target[1] # -2
        error = ref - self.robot_position_y

        kp = 1000
        K_output = -kp * error

        K_output = 1500 + K_output

        K_output = self.bound_limit(K_output, 1100, 1900)
        error = self.bound_limit(error, -1000, 1000)

        # set speed thrusters
        self.RCOut_msg = RCOut()
        self.RCOut_msg.header.stamp = rospy.Time.now()
        self.RCOut_msg.header.frame_id = "0"
        self.RCOut_msg.channels = [K_output, K_output, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
        self.mavros_vel_pub.publish(self.RCOut_msg)

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

def main(args):
    rospy.init_node('robdos_waypoint_controller', anonymous=True)
    ic = robdos_waypoint_controller()


if __name__ == '__main__':
    main(sys.argv)