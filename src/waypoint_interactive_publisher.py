#!/usr/bin/env python

import rospy
import copy
import sys

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from mavros_msgs.msg import WaypointList, Waypoint

from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class robdos_waypoint_interactive_publisher:
    def __init__(self):
        self.br = None
        self.server = None

        self.rate_config = rospy.get_param('~rate', 10.0)
        self.current_waypoint_position_x = rospy.get_param('~current_waypoint_position_x', 0.0)
        self.current_waypoint_position_y = rospy.get_param('~current_waypoint_position_y', 0.0)

        self.g_set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

        # mavros marker setup
        # configure waypoint publisher
        self.mavros_waypoint_pub = rospy.Publisher("/mavros/mission/waypoints",  WaypointList, queue_size=1)
        self.waypoint_msg = WaypointList()

        # define rate
        self.rate = rospy.Rate(self.rate_config)

        self.br = TransformBroadcaster()

        self.server = InteractiveMarkerServer("waypoint_interactive_publisher")

        position = Point( self.current_waypoint_position_x, self.current_waypoint_position_y, 0)
        self.make_auv_waypoint_Marker( position )

        self.server.applyChanges()

        # infinity loop
        while not rospy.is_shutdown():
            self.update_waypoints()

            self.rate.sleep()

    def processFeedback(self, feedback ):
        '''
        Process Callback of user interaction: get new marker position
        :param feedback: info about new position of marker
        :return:
        '''
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        mp = ""

        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        #if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        #    rospy.loginfo( s + ": pose changed")

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )
            self.current_waypoint_position_x = feedback.mouse_point.x
            self.current_waypoint_position_y = feedback.mouse_point.y

        self.server.applyChanges()

    def make_maker(self, msg ):
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.85
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def make_auv_waypoint_Marker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "fcu"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "AUV"
        int_marker.description = "AUV"

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(copy.deepcopy(control))

        # make a box which also moves in the plane
        control.markers.append( self.make_maker(int_marker) )
        control.always_visible = True
        int_marker.controls.append(control)

        # we want to use our special callback function
        self.server.insert(int_marker, self.processFeedback)

    def update_waypoints(self):
        '''
        Creates and publishes waypoint list using mavros/waypoints, also update position of model (marker) in Gazebo
        :return:
        '''
        self.waypoint_msg = WaypointList()

        # Publish mavros/waypoints
        way_tmp = Waypoint()
        way_tmp.frame = 0
        way_tmp.command = 16
        way_tmp.is_current = True
        self.waypoint_msg.waypoints.append(way_tmp)

        way_tmp = Waypoint()
        way_tmp.frame = 0
        way_tmp.command = 22
        way_tmp.is_current = False
        way_tmp.autocontinue = True
        way_tmp.param2 = 1
        way_tmp.param3 = 0
        way_tmp.param4 = 0
        way_tmp.x_lat = self.current_waypoint_position_x
        way_tmp.y_long = self.current_waypoint_position_y
        way_tmp.z_alt = 0
        self.waypoint_msg.waypoints.append(way_tmp)

        self.mavros_waypoint_pub.publish(self.waypoint_msg)

        self.update_waypoint_model()

    def update_waypoint_model(self):
        pose = Pose()

        pose.position.x = self.current_waypoint_position_x
        pose.position.y = self.current_waypoint_position_y
        pose.position.z = 0

        state = ModelState()
        state.model_name = "reference_waypoint"
        state.pose = pose

        try:
            #pass
            ret = self.g_set_state(state)

        except Exception, e:
            rospy.logerr('Error on calling service: %s',str(e))

def main(args):
    rospy.init_node('waypoint_interactive_publisher', anonymous=True)
    ic = robdos_waypoint_interactive_publisher()

if __name__ == '__main__':
    main(sys.argv)

