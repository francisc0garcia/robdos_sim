/*
* Based on ROS Diff Drive
 **/

#ifndef SUBDRIVE_PLUGIN_HH
#define SUBDRIVE_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "mavros_msgs/RCOut.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/OverrideRCIn.h"

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

    class GazeboRosSubDrive : public ModelPlugin {

    public:
        GazeboRosSubDrive();

        ~GazeboRosSubDrive();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void cmdVelCallback (const mavros_msgs::OverrideRCIn::ConstPtr& cmd_msg );

        void Reset();

    protected:
        virtual void UpdateChild();
        virtual void FiniChild();

    private:
        int yaw, forward;
        // Update Rate
        double update_rate_, update_period_, scale_controller;
        double temp_vel_x, temp_vel_y, temp_vel_z, temp_x, temp_y, temp_z, prev_temp_x, prev_temp_y, prev_temp_z;
        bool alive_;
        std::string odometry_topic_;

        GazeboRosPtr gazebo_ros_;
        physics::ModelPtr parent;
        physics::JointPtr joint_motor_left, joint_motor_right, joint_motor_center;
        event::ConnectionPtr update_connection_;

        // ROS STUFF
        ros::Publisher odometry_publisher_;
        ros::Subscriber cmd_vel_subscriber_;
        ros::CallbackQueue queue_;

        nav_msgs::Odometry odom_;
        nav_msgs::Odometry pose_sub;
        geometry_msgs::Pose2D pose_encoder_;
        common::Time last_update_time_;
        common::Time last_odom_update_;
        boost::mutex lock;
        boost::thread callback_queue_thread_;

        void QueueThread();
    };
}

#endif

