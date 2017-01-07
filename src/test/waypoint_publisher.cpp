#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/Waypoint.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_1_only_robot");

    ros::NodeHandle n;

    ros::Rate r(10);
    mavros_msgs::WaypointList list_test;




    while ( ros::ok() )
    {
        ros::spinOnce(); // refresh once
        r.sleep();
    }

    return 0;
}


