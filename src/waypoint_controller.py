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
from robdos_sim.msg import state


class robdos_waypoint_controller:
	def __init__(self):

		# define rate of  10 hz
		self.rate = rospy.Rate(50.0)

		# init variables for odometry
		[self.robot_position_x, self.robot_position_y, self.robot_position_z] = [0, 0, 0]

		# mavros velocity publisher
		self.mavros_vel_pub = rospy.Publisher("/mavros/rc/out",  RCOut, queue_size=1)
		self.RCOut_msg = RCOut()

######################################################SUBSCRIBERS########################################################
		# create subscriber for robot localization
		#self.sub_localization = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.process_localization_message, queue_size=1)
		self.sub_localization = rospy.Subscriber('/robdos/odom', Odometry, self.process_localization_message, queue_size=1)
		self.robot_roll = 0
		self.robot_pitch = 0
		self.robot_yaw = 0
		# create subscriber for mavros/waypointlist
		self.sub_waypoint_list = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.process_waypoint_message, queue_size=1)
		self.list_points = []
		self.is_waypoint_loaded = False
		self.current_target = [0, 0, 0]
		# create subscriber for the state of the machine
		self.sub_state = rospy.Subscriber('robdos_sim/state', state, self.process_state_message, queue_size=1)
		self.stateCurrent = 2
		# create subscriber in order to have the same output in the teleoperation mode
		self.sub_rc = rospy.Subscriber('/mavros/rc/out_mavros', RCOut, self.process_rc_message, queue_size=1)
		self.RCOut_msgCurrent = RCOut()

######################################################CONTROLLER########################################################
		# infinity loop
		while not rospy.is_shutdown():
			self.update_controller()
			self.rate.sleep()

#################################################SUBSCRIBER LOCALIZATION####################################################
	# update position and orientation of robot (odometry)
	def process_localization_message(self, odometry_msg):
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

#################################################SUBSCRIBER WAYPOINTS########################################################
	# update list of waypoints
	def process_waypoint_message(self, waypoint_list_msg):
		self.list_points = []

		for waypoint in waypoint_list_msg.waypoints:
			# set last value to False: means not reached jet.
			self.list_points.append( [waypoint.x_lat, waypoint.y_long, waypoint.z_alt, waypoint.is_current] )

#################################################SUBSCRIBER STATE############################################################
	def process_state_message(self, state_msg):
		self.stateCurrent = state_msg.state

#################################################SUBSCRIBER RC########################################################
	def process_rc_message(self, rc_out_msg):
		self.RCOut_msgCurrent = rc_out_msg

######################################################CONTROLLER##############################################################
	def update_controller(self):
		if self.stateCurrent == 1:
			self.RCOut_msg = RCOut()
			self.RCOut_msg.header.stamp = rospy.Time.now()
			self.RCOut_msg.header.frame_id = "0"
			self.RCOut_msg.channels = RCOut_msgCurrent.channels

		elif self.stateCurrent == 2:
			if len(self.list_points) > 0:
				point = self.list_points[1]
				self.current_target = [point[0], point[1], point[2] ]

			# update controller
			desired_yaw = math.atan2(self.current_target[0], self.current_target[1])



			yaw_error = desired_yaw - self.robot_yaw

			#if math.fabs(yaw_error) > 1:
			if yaw_error > 0.1:

				kp = 1000

				K_output =  -kp * yaw_error

				K_LM = 1500 + K_output
				K_RM = 1500 - K_output

				K_LM = self.bound_limit(K_LM, 1100, 1900)
				K_RM = self.bound_limit(K_RM, 1100, 1900)

				# set speed thrusters
				self.RCOut_msg = RCOut()
				self.RCOut_msg.header.stamp = rospy.Time.now()
				self.RCOut_msg.header.frame_id = "0"
				self.RCOut_msg.channels = [K_LM, K_RM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]


			else:
				distance_x = self.current_target[0] - self.robot_position_x
				distance_y = self.current_target[1] - self.robot_position_y

				kp = 1000
				error = math.sqrt(math.pow(distance_x, 2.0) + math.pow(distance_y, 2.0))

				if distance_y > 0:
					if distance_x > 0:
						K_output = -kp * error

					elif distance_x < 0:
						K_output = -kp * error

					elif distance_x == 0:
						K_output = -kp * error

				elif distance_y < 0:
					if distance_x > 0:
						K_output = kp * error

					elif distance_x < 0:
						K_output = kp * error

					elif distance_x == 0:
						K_output = kp * error

				elif distance_y == 0:
					if distance_x > 0:
						K_output = -kp * error

					elif distance_x < 0:
						K_output = kp * error

					elif distance_x == 0:
						K_output = 0

				K_output = 1500 + K_output

				K_output = self.bound_limit(K_output, 1100, 1900)

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