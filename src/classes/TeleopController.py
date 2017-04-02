#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import math
import numpy as np
from mavros_msgs.msg import WaypointList, Waypoint, OverrideRCIn
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from robdos_sim.cfg import controllerOrientationConfig
from robdos_sim.msg import StateEvent


from PID import *

def arduino_map(x, inmin, inmax, outmin, outmax):
        return (x - inmin) * (outmax - outmin) / (inmax - inmin) + outmin

class RCChan(object):
    def __init__(self, name, chan, min_pos=-1.0):
        self.name = name
        self.chan = chan
        self.min = 1100
        self.max = 1900
        self.min_pos = min_pos

    def load_param(self):
        self.chan = rospy.get_param("~rc_map/" + self.name, self.chan)
        self.min = rospy.get_param("~rc_min/" + self.name, self.min)
        self.max = rospy.get_param("~rc_max/" + self.name, self.max)

    def calc_us(self, pos):
        # warn: limit check
        return arduino_map(pos, self.min_pos, 1.0, self.min, self.max)

class TeleopController:
    def __init__(self, debug=False):

        # event publisher
        self.event_pub = rospy.Publisher("/robdos/stateEvents", StateEvent, queue_size=1)
        self.event_msg = StateEvent()
        self.debug = debug

        # mavros velocity publisher
        self.mavros_vel_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.RCOR_msg = OverrideRCIn()

        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.process_joy_message, queue_size=1)

        self.forward = None
        self.pitch = None
        self.yaw = None
        self.throttle = None

        self.axes_map = {
            'forward': 3,
            'pitch': 4,
            'yaw': 2,
            'throttle': 1
        }

        self.axes_scale = {
            'forward': 1.0,
            'pitch': 1.0,
            'yaw': 1.0,
            'throttle': 1.0
        }

        # Botones de Joystick
        self.button_map = {
            'arm1' : 10,
            'arm2' : 11,
            'disarm1' : 8,
            'disarm2' : 9,
            'takeoff': 2,
            'land': 3,
            'enable': 4
        }

        # Canales del topic /mavros/rc/Override
        self.rc_channels = {
            'forward': RCChan('forward', 5),
            'pitch': RCChan('pitch', 1),
            'yaw': RCChan('yaw', 3),
            'throttle': RCChan('throttle', 6)
        }

    def register(self):
        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.process_joy_message, queue_size=1)

    def get_axis(self, j, n):
        return j.axes[self.axes_map[n]] * self.axes_scale[n]

    def set_chan(self, n, v):
        ch = self.rc_channels[n]
        self.RCOR_msg.channels[ch.chan] = ch.calc_us(v)

    def process_joy_message(self, joy_msg):
        if self.debug:
            rospy.loginfo('teleoperation controller')

        self.RCOR_msg.channels = [0, 0, 0, 0, 0, 0, 0, 0]

        self.forward = self.get_axis(joy_msg, 'forward')
        self.pitch = self.get_axis(joy_msg, 'pitch')
        self.yaw = self.get_axis(joy_msg, 'yaw')
        self.throttle = self.get_axis(joy_msg, 'throttle')

        self.set_chan('forward', self.forward)
        self.set_chan('pitch', self.pitch)
        self.set_chan('yaw', self.yaw)
        self.set_chan('throttle', self.throttle)

        self.mavros_vel_pub.publish(self.RCOR_msg)


    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)
