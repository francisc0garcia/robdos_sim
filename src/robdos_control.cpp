////////////////////////////////////////////SUBSCRIBER WAYPOINTS/////////////////////////////
//mavros/mission/waypoints

/////////////////////////////////////////SUBSCRIBER LOCALIZATION/////////////////////////////
//mavros/local_position/pose

/////////////////////////////////////////SUBSCRIBER LOCALIZATION/////////////////////////////
//mavros/rc/out_mavros

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCOut.h>

/////////////////////////////////////////WAYPOINTS///////////////////////////////////////////
float param1 = 0, param2 = 0, param3 = 0, param4 = 0;
double x_lat = 0, y_long = 0, z_alt = 0;

std::string node_name;

void waypointsMessage(const mavros_msgs::RCOut &waypointsDesired) {

    param1 = waypointsDesired.param1;
    param2 = waypointsDesired.param2;
    param3 = waypointsDesired.param3;
    param4 = waypointsDesired.param4;
    x_lat = waypointsDesired.x_lat;
    y_long = waypointsDesired.y_lat;
    z_alt = waypointsDesired.z_lat;
    return 0;
}

//////////////////////////////////LOCALIZATION GPS///////////////////////////////////////////
float xCurrentPosition = 0, CurrentPosition = 0, zCurrentPosition = 0, yCurrentPosition=0;

void localizationMessage(const geometry_msgs::PoseStamped &localizationCurrent) {
    //http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    xCurrentPosition = (float)localizationCurrent.pose.position.x;
    yCurrentPosition = (float)localizationCurrent.pose.position.y;
    zCurrentPosition = (float)localizationCurrent.pose.position.z;
}

/////////////////////////////////////CURRENT RCOut///////////////////////////////////////////
float channelsRCOutCurrent[15]; // = 1500;

void mavrosMessage(const mavros_msgs::RCOut &RCOutCurrent) {
    channelsRCOutCurrent[0] = RCOutCurrent.channels[0];
    channelsRCOutCurrent[1] = RCOutCurrent.channels[1];
}

/////////////////////////////////////STATE MACHINE///////////////////////////////////////////
void estado() {

}

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////  MAIN  //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

    ros::init(argc, argv, node_name);     // Note node_name can be overidden by launch file
    ros::NodeHandle node;

///////////////////////////////////////SUBSCRIBERS///////////////////////////////////////////
    out_waypoints = node.subscribe("/mavros/mission/waypoints", 10, waypointsMessage);
    out_localization = node.subscribe("/mavros/local_position/pose", 10, localizationMessage);
    out_mavros = node.subscribe("/mavros/rc/out_mavros", 10, mavrosMessage);
    maquinaEstados = node.subscribe("/mavros/rc/out_mavros", 10, estado);

///////////////////////////////////////PUBLISHER/////////////////////////////////////////////
    pub = node.advertise<mavros_msgs::OverrideRCOut>("mavros/rc/out", 10);

    /////////////////////////////////////CONTROL GUIDED/////////////////////////////////////////
    if (estado == GUIDED) {
        mavros_msgs::RCOut rc_out;
        rc_out.channels = channels;
        pub.publish(rc_out);
    }

////////////////////////////////////CONTROL WAYPOINTS/////////////////////////////////////////
    else if (estado == WAYPOINT) {
        mavros_msgs::RCOut rc_out;
        float k_output_Right = 1500;
        float k_output_Left = 1500;
        float k_output_Vertical = 1500;
        float error = 0;

        float kp = 800;
        float errorRight = 0, errorLeft = 0;

        errorLeft = 0;
        errorRight = -errorLeft;

        k_output_Right = -kp * error;
        k_output_Right = k_output_Right + 1500;

        k_output_Left = -kp * error;
        k_output_Left = k_output_Left + 1500;

        if (k_output_Right < 1100) { k_output_Right = 1100; }
        else if (k_output_Right > 1900) { k_output_Right = 1900; }
        if (k_output_Left < 1100) { k_output_Left = 1100; }
        else if (k_output_Left > 1900) { k_output_Left = 1900; }

        rc_out.channels[0] = (unsigned short)k_output_Right;
        rc_out.channels[1] = (unsigned short)k_output_Left;

        pub.publish(rc_out);
    }

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
