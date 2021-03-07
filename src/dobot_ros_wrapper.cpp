#include "dobot_magician_driver/dobot_ros_wrapper.h"

DobotRosWrapper::DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port)
    : _nh(nh)
    , _pn(pn)
    , _rate(100)
{
    port_ = port;
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    end_effector_state_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("end_effector_state", 1);
}

DobotRosWrapper::~DobotRosWrapper()
{
    update_state_thread_->join();
}

void DobotRosWrapper::init()
{
    // Initialise Dobot
    driver_ = new DobotDriver(port_);
    driver_->initialiseDobot();

    ROS_INFO("DobotRosWrapper: this thread will sleep for Dobot initialise sequence");
    std::this_thread::sleep_for(std::chrono::seconds(30));
    ROS_INFO("DobotRosWrapper: this thread will now wake up");
}

void DobotRosWrapper::run()
{
    update_state_thread_ = new std::thread(&DobotRosWrapper::updateStateLoop, this);
    robot_control_thread_ = new std::thread(&DobotRosWrapper::robotControlLoop, this);
}

void DobotRosWrapper::updateStateLoop()
{
    sensor_msgs::JointState joint_angle_msg;
    geometry_msgs::PoseStamped cart_pos_msg;

    ROS_INFO("DobotRosWrapper: data from Dobot is now being published");
    ROS_DEBUG("DobotRosWrapper: update_state_thread started");

    ros::Time ros_time;

    bool robot_joints_at_joint_targets = false;
    bool robot_end_effector_at_target_pose = false;

    while(ros::ok())
    {
        ros_time = ros::Time::now();
        driver_->getCurrentConfiguration(latest_end_effector_pos_, latest_joint_angles_);

        if(latest_end_effector_pos_.size() == 4 && latest_joint_angles_.size() == 4)
        {
            int total_joints_correct = 0;
            int total_cart_correct = 0;

            // Update joint configuration
            joint_ang_msg.position.clear();
            for(int i = 0; i < latest_joint_angles_.size(); ++i)
            {
                joint_ang_msg.position.push_back(latest_joint_angles_[i]*M_PI/180);

                if(abs(latest_joint_angles_[i]*M_PI/180 - target_joint_data_[i]) <= JOINT_ERROR)
                {
                    total_joints_correct++;
                }
            }

            // Compare current configuration with target configuration

            if(total_joints_correct == 4)
            {
                robot_joints_at_joint_targets = true;
            }
            else
            {
                robot_joints_at_joint_targets = false;
            }

            // Update current translation

            cart_pos_msg.pose.position.x = latest_end_effector_pos_[0]/1000;
            cart_pos_msg.pose.position.y = latest_end_effector_pos_[1]/1000;
            cart_pos_msg.pose.position.z = latest_end_effector_pos_[2]/1000;

            // Compare current translation and target translation

            if(sqrt(pow(cart_pos_msg.pose.position.x - target_end_effector_pose_data_.pose_data.position.x,2) + 
                    pow(cart_pos_msg.pose.position.y - target_end_effector_pose_data_.pose_data.position.y,2) + 
                    pow(cart_pos_msg.pose.position.z - target_end_effector_pose_data_.pose_data.position.z,2)) <= LINEAR_ERROR)
            {
                total_cart_correct = 3;
            }

            // Update current orientation

            if(latest_end_effector_pos_[3] < 0.0)
            {
                latest_end_effector_pos_[3] = latest_end_effector_pos_ + 360.0;
            }

            cart_pos_msg.pose.orientation.w = (cos(latest_end_effector_pos_[3]*0.5*M_PI/180.0)); //assumes only changes in yaw
            cart_pos_msg.pose.orientation.x = 0;
            cart_pos_msg.pose.orientation.y = 0;
            cart_pos_msg.pose.orientation.z = sqrt(1-pow(cart_pos_msg.pose.orientation.w,2));

            // Compare current orientation and target orientation

            if(abs(tf::getYaw(cart_pos_msg.pose.orientation) - tf::getYaw(target_end_effector_pose_data_.pose_data.orientation)) <= ORIENTATION_ERROR)
            {
                total_cart_correct++
            }

            if(total_cart_correct == 4)
            {
                robot_end_effector_at_target_pose = true;
            }
            else
            {
                robot_end_effector_at_target_pose = false;
            }

            if(robot_end_effector_at_target_pose || robot_joints_at_joint_targets)
            {
                robot_at_target_ = true;
                robot_in_motion_ = false;
            }
            else
            {
                robot_at_target_ = false;
                robot_in_motion_ = true;
            }

            // Update robot state to ROS
            cart_pos_msg.header.stamp = ros_time;
            joint_ang_msg.header.stamp = ros_time;
            joint_state_pub_.publish(joint_ang_msg);
            end_effector_state_pub_.publish(cart_pos_msg);
        }

        rate_.sleep();
    }
}

void DobotRosWrapper::robotControlLoop()
{
    while(ros::ok())
    {
        // Control loop
        // If receives target joints and robot is not moving, move joints
        if(target_joint_data_.received && !robot_in_motion_)
        {
            moveToTargetJoints();
        }
        // If receives target pose and robot is not moving, move to pose
        if(target_end_effector_pose_data_.received && !robot_in_motion_)
        {
            moveToTargetEndEffectorPose();
        }
        
    }
}

void DobotRosWrapper::jointTargetCallback(const sensor_msgs::JointState& msg)
{
    ROS_INFO("New target joint configurations received !!");
    
    target_joint_data_.mtx.lock();
    target_joint_data_.position = msgs->position;   
    target_joint_data.mtx.unlock();
}

void DobotRosWrapper::endEffectorTargetPoseCallback(const sensor_msgs::Pose& msg)
{
    ROS_INFO("New target end effector pose received !!");

    target_end_effector_pose_data_.mtx.lock();
    target_end_effector_pose_data_.pose_data = msgs->poses;
    target_end_effector_pose_data_.mtx.unlock();
}

bool DobotRosWrapper::moveToTargetJoints()
{
    std::vector<float> target_joints;
    target_joint_data_.mtx.lock();
    for(int i = 0; i < 4; i++)
    {
        target_joints.push_back(target_joint_data_.position[i] * 180 / M_PI);
    }   
    target_joint_data_.mtx.unlock();

    if(driver_->setJointAngles(target_joints))
    {
        robot_in_motion_ = true;
        robot_at_target_ = false;

        return true;
    }
    else
    {
        return false;
    }
}

bool DobotRosWrapper::moveToTargetEndEffectorPose()
{
    std::vector<float> target_points;
    target_end_effector_pose_data_.mtx.lock();

    target_points.push_back(target_joint_data_.pose_data.position.x * 1000);
    target_points.push_back(target_joint_data_.pose_data.position.y * 1000);
    target_points.push_back(target_joint_data_.pose_data.position.z * 1000);
    target_points.push_back(tf::getYaw(target_joint_data_.pose_data.orientation));
    
    target_end_effector_pose_data_.mtx.unlock();

    if(driver_->setCartesianPos(target_points))
    {
        robot_in_motion_ = true;
        robot_at_target_ = false;

        return true;
    }
    else
    {
        return false;
    }
}

