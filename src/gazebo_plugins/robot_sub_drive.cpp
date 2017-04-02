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

        gazebo_ros_->getParameter<double>(scale_controller, "scale_controller", 0.2);
        scale_controller = scale_controller * 0.001;

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
                ros::SubscribeOptions::create<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1,
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

    void GazeboRosSubDrive::cmdVelCallback ( const mavros_msgs::OverrideRCIn::ConstPtr& cmd_msg )
    {
        boost::mutex::scoped_lock scoped_lock ( lock );

        // yaw: channel 3
        // forward: Channel 5
        yaw = cmd_msg->channels[3];
        forward = cmd_msg->channels[5];
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
            _pose.pos.z = -1;

            //for testing: (fix x coordinate, robot only moves in y)

            //_pose.pos.x = 0;
            //_yaw = 1.571;

            _pose.rot.SetFromEuler(_roll, _pith, _yaw);

            this->parent->SetWorldPose(_pose);

            double vel_left = 0;
            double vel_right = 0;

            if (yaw > 1500){
                vel_left = -(yaw - 1500) ;
                vel_right = yaw - 1500;
            }else if (yaw < 1500){
                vel_left = 1500 - yaw;
                vel_right = -(1500 - yaw) ;
            }

            if (forward > 1500){
                vel_left = forward - 1500;
                vel_right = forward - 1500;
            }else if (forward < 1500){
                vel_left = -(forward - 1500);
                vel_right = -(forward - 1500);
            }

            joint_motor_left->SetVelocity(0, vel_left * scale_controller);
            joint_motor_right->SetVelocity(0, vel_right * scale_controller);
            joint_motor_center->SetVelocity(0, 0);

            // for debugging
            //ROS_INFO("vel: %f,  %f", vel_left , vel_right);

            // Compute odometry
            temp_x = this->parent->GetWorldPose().pos.x;
            temp_y = this->parent->GetWorldPose().pos.y;
            temp_z = this->parent->GetWorldPose().pos.z;

            odom_.pose.pose.position.x = this->parent->GetWorldPose().pos.x;
            odom_.pose.pose.position.y = this->parent->GetWorldPose().pos.y;
            odom_.pose.pose.position.z = this->parent->GetWorldPose().pos.z;

            odom_.pose.pose.orientation.x = this->parent->GetWorldPose().rot.x;
            odom_.pose.pose.orientation.y = this->parent->GetWorldPose().rot.y;
            odom_.pose.pose.orientation.z = this->parent->GetWorldPose().rot.z;
            odom_.pose.pose.orientation.w = this->parent->GetWorldPose().rot.w;

            temp_vel_x = (temp_x - prev_temp_x) / seconds_since_last_update;
            temp_vel_y = (temp_y - prev_temp_y) / seconds_since_last_update;

            odom_.twist.twist.linear.x = sqrt( pow(temp_vel_x, 2.0) + pow(temp_vel_y, 2.0) ) ;
            odom_.twist.twist.linear.y = 0;
            odom_.twist.twist.linear.z = 0;

            // set header
            odom_.header.stamp = ros::Time::now();
            odom_.header.frame_id = "fcu";
            odom_.child_frame_id = "fcu";

            odometry_publisher_.publish(odom_);

            last_update_time_ = parent->GetWorld()->GetSimTime();
            prev_temp_x = temp_x;
            prev_temp_y = temp_y;
            prev_temp_z = temp_z;
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

