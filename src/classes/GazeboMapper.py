#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import math
import numpy as np
from mavros_msgs.msg import WaypointList, Waypoint, RCOut, OverrideRCIn
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from robdos_sim.cfg import controllerOrientationConfig
from robdos_sim.msg import StateEvent


from PID import *

class GazeboMapper:
    def __init__(self):

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("/mavros/rc/out", RCOut, queue_size=1)
        self.RCOut_msg = RCOut()

        self.RCOR_subscriber = rospy.Subscriber('/mavros/rc/override', OverrideRCIn, self.override_cb, queue_size=1)

    def doSymmetric(self, K):
        dif = abs(K-1500)
        if K >= 1500:
            return 1500 - dif
        elif K < 1500 + dif:
            return 1500 + dif

    def override_cb(self, or_msg):

        self.RCOut_msg = RCOut()

        dif = or_msg.channels[3]-1500

        if dif >= 0:
            K_RM = or_msg.channels[5] + dif
            K_LM = or_msg.channels[5] - dif
        else:
            K_RM = or_msg.channels[5] - abs(dif)
            K_LM = or_msg.channels[5] + abs(dif)

        K_RM = self.doSymmetric(K_RM)
        K_LM = self.doSymmetric(K_LM)
 
        self.RCOut_msg.channels = [self.bound_limit(K_LM, 1100, 1900), self.bound_limit(K_RM, 1100, 1900), \
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.mavros_vel_pub.publish(self.RCOut_msg)


    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)
