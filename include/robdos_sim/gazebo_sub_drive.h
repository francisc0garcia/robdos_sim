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
        void cmdVelCallback (const mavros_msgs::RCOut::ConstPtr& cmd_msg );

        void Reset();

    protected:
        virtual void UpdateChild();

        virtual void FiniChild();

    private:
        GazeboRosPtr gazebo_ros_;
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;


        // ROS STUFF
        ros::Publisher odometry_publisher_;
        nav_msgs::Odometry odom_;

        ros::Subscriber cmd_vel_subscriber_;

        boost::mutex lock;

        std::string odometry_topic_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        void QueueThread();

        nav_msgs::Odometry pose_sub;

        int thruster_left, thruster_right, thruster_center;

        // Update Rate
        double update_rate_, update_period_;
        bool alive_;
        common::Time last_update_time_;

        geometry_msgs::Pose2D pose_encoder_;
        common::Time last_odom_update_;

        physics::JointPtr joint_motor_left, joint_motor_right, joint_motor_center;
    };
}

#endif

