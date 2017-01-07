#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import WaypointList, Waypoint
from marker_generator import *

class robdos_waypoint_publisher:

    def __init__(self):
        # define rate of  10 hz
        self.rate = rospy.Rate(10.0)

        self.list_points = []

        # markers setup
        # we use markers for RVIZ visualization:
        self.waypoint_pub = rospy.Publisher("/robdos/makers_waypoints", Marker, queue_size=1)
        self.marker_gen = MarkerGenerator()
        self.marker_gen.ns = '/robdos'
        self.marker_gen.type = Marker.SPHERE_LIST
        self.marker_gen.scale = [.5]*3
        self.marker_gen.frame_id = 'fcu'

        # mavros marker setup
        # configure waypoint publisher
        self.mavros_waypoint_pub = rospy.Publisher("/mavros/mission/waypoints",  WaypointList, queue_size=1)
        self.waypoint_msg = WaypointList()

        # call function for adding fixed points for testing
        self.add_points_to_list()

        # infinity loop
        while not rospy.is_shutdown():
            self.update_waypoints()
            self.rate.sleep()


    def add_points_to_list(self):
        # create some fixed waypoints (X, Y, Z)

        # add manually points
        self.list_points.append([0, 1, 0])
        self.list_points.append([0, 3, 0])
        self.list_points.append([0, 5, 0])

        # add more points
        for i in range(4):
            self.list_points.append([-i*2, -i*2, 1])


    def update_waypoints(self):
        self.waypoint_msg = WaypointList()

        # Publish markers for rviz visualization
        self.marker_gen.color = [0, 1, 0, 0.8]
        self.marker_gen.scale = [0.3, 0.3, 1]
        self.marker_gen.type = Marker.SPHERE_LIST

        m = self.marker_gen.marker(points=self.list_points, id=0)

        self.waypoint_pub.publish(m)

        # Publish mavros/waypoints
        way_tmp = Waypoint()
        way_tmp.frame = 0
        way_tmp.command = 16
        way_tmp.is_current = True
        self.waypoint_msg.waypoints.append(way_tmp)

        for point in self.list_points:
            way_tmp = Waypoint()
            way_tmp.frame = 0
            way_tmp.command = 22
            way_tmp.is_current = False
            way_tmp.autocontinue = True
            way_tmp.param2 = 1
            way_tmp.param3 = 0
            way_tmp.param4 = 0
            way_tmp.x_lat = point[0]
            way_tmp.y_long = point[1]
            way_tmp.z_alt = 0
            self.waypoint_msg.waypoints.append(way_tmp)

        self.mavros_waypoint_pub.publish(self.waypoint_msg)


def main(args):
    rospy.init_node('robdos_waypoint_publisher', anonymous=True)
    ic = robdos_waypoint_publisher()

if __name__ == '__main__':
    main(sys.argv)