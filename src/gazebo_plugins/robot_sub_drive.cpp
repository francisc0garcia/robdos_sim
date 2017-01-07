#include <ros/ros.h>
#include <algorithm>
#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>
#include <robdos_sim/gazebo_sub_drive.h>


namespace gazebo {
    GazeboRosSubDrive::GazeboRosSubDrive() {}

// Destructor
    GazeboRosSubDrive::~GazeboRosSubDrive() {}

// Load the controller
    void GazeboRosSubDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

        this->parent = _parent;

        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "sub_drive"));
        gazebo_ros_->isInitialized();

        joint_motor_left, joint_motor_right, joint_motor_center;

        // get link elements from sdf model
        joint_motor_left = gazebo_ros_->getJoint(parent, "joint_motor_left", "_joint_motor_left");
        joint_motor_right = gazebo_ros_->getJoint(parent, "joint_motor_right", "_joint_motor_right");
        joint_motor_center = gazebo_ros_->getJoint(parent, "joint_motor_center", "_joint_motor_center");

        // Initialize update rate
        this->update_period_ = 0.10;

        last_update_time_ = parent->GetWorld()->GetSimTime();

        // creates a publisher for sending odometry data
        odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>("/robdos/odom", 1);

        alive_ = true;

        // create subscriber for thrusters desired commands
        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<mavros_msgs::RCOut>("/mavros/rc/out", 1,
                                                                    boost::bind(&GazeboRosSubDrive::cmdVelCallback, this, _1),
                                                                    ros::VoidPtr(), &queue_);

        cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);

        // start custom queue for diff drive
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&GazeboRosSubDrive::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosSubDrive::UpdateChild, this));

    }

    void GazeboRosSubDrive::cmdVelCallback ( const mavros_msgs::RCOut::ConstPtr& cmd_msg )
    {
        boost::mutex::scoped_lock scoped_lock ( lock );

        // read only first 3 values
        thruster_left = cmd_msg->channels[0];
        thruster_right = cmd_msg->channels[1];
        thruster_center = cmd_msg->channels[2];
    }

    void GazeboRosSubDrive::Reset() {
        last_update_time_ = parent->GetWorld()->GetSimTime();
    }

// Update the controller
    void GazeboRosSubDrive::UpdateChild() {
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            math::Pose _pose;
            _pose.pos.x = this->parent->GetWorldPose().pos.x;
            _pose.pos.y = this->parent->GetWorldPose().pos.y;

            double _roll, _pith, _yaw;

            _roll = this->parent->GetWorldPose().rot.GetAsEuler().x;
            _pith = this->parent->GetWorldPose().rot.GetAsEuler().y;
            _yaw = this->parent->GetWorldPose().rot.GetAsEuler().z;

            // fix to plane z ( force lineal movement)
            _pith = 0;
            _roll = 0;
            _pose.pos.z = 0;

            //for testing: (fix x coordinate, robot only moves in y)
            _pose.pos.x = 0;
            _yaw = 1.571;

            _pose.rot.SetFromEuler(_roll, _pith, _yaw);

            this->parent->SetWorldPose(_pose);

            // set motor velocities
            double vel_left = (1500.0 - thruster_left ) / 500.0;
            double vel_right = (1500.0 - thruster_right ) / 500.0;

            joint_motor_left->SetVelocity(0, vel_left);
            joint_motor_right->SetVelocity(0, vel_right);
            joint_motor_center->SetVelocity(0, 0);

            // for debugging
            //ROS_INFO("vel: %f,  %f", vel_left , vel_right);

            // Compute odometry
            odom_.pose.pose.position.x = this->parent->GetWorldPose().pos.x;
            odom_.pose.pose.position.y = this->parent->GetWorldPose().pos.y;
            odom_.pose.pose.position.z = this->parent->GetWorldPose().pos.z;

            odom_.pose.pose.orientation.x = this->parent->GetWorldPose().rot.x;
            odom_.pose.pose.orientation.y = this->parent->GetWorldPose().rot.y;
            odom_.pose.pose.orientation.z = this->parent->GetWorldPose().rot.z;
            odom_.pose.pose.orientation.w = this->parent->GetWorldPose().rot.w;

            // set header
            odom_.header.stamp = ros::Time::now();
            odom_.header.frame_id = "fcu";
            odom_.child_frame_id = "fcu";

            odometry_publisher_.publish(odom_);

            last_update_time_ = parent->GetWorld()->GetSimTime();
        }
    }

// Finalize the controller
    void GazeboRosSubDrive::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }


    void GazeboRosSubDrive::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }

        this->Reset();
        this->FiniChild();
    }


    GZ_REGISTER_MODEL_PLUGIN (GazeboRosSubDrive)
}

