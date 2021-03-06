#include "dobot_magician_driver/dobot_ros_wrapper.h"

DobotRosWrapper::DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port)
    : _nh(nh)
    , _pn(pn)
    , _rate(100)
{
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    end_effector_state_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("end_effector_state", 1);

    ROS_INFO("DobotRosWrapper: this thread will sleep for Dobot initialise sequence");
    std::this_thread::sleep_for(std::chrono::seconds(30));
    ROS_INFO("DobotRosWrapper: this thread will now wake up");

    // _update_state_thread = new std::thread(&DobotRosWrapper::updateStateLoop, this);
}

DobotRosWrapper::~DobotRosWrapper()
{
    update_state_thread_->join();
}

void DobotRosWrapper::updateStateLoop()
{
    sensor_msgs::JointState joint_angle_msg;
    geometry_msgs::PoseStamped cart_pos_msg;

    ROS_INFO("DobotRosWrapper: data from Dobot is now being published");
    ROS_DEBUG("DobotRosWrapper: update_state_thread started");

    ros::Time ros_time;
    while(ros::ok())
    {
        ros_time = ros::Time::now();
        driver_->getCurrentConfiguration(latest_end_effector_pos_, latest_joint_angles_);

        if(latest_end_effector_pos_.size() == 4 && latest_joint_angles_.size() == 4)
        {
            // Update joint configuration
            joint_ang_msg.position.clear();
            for(int i = 0; i < latest_joint_angles_.size(); ++i){
                joint_ang_msg.position.push_back(latest_joint_angles_[i]*M_PI/180);
            }

            cart_pos_msg.pose.position.x = latest_end_effector_pos_[0]/1000;
            cart_pos_msg.pose.position.y = latest_end_effector_pos_[1]/1000;
            cart_pos_msg.pose.position.z = latest_end_effector_pos_[2]/1000;

            if(latest_end_effector_pos_[3] < 0.0)
            {
                latest_end_effector_pos_[3] = latest_end_effector_pos_ + 360.0;
            }

            cart_pos_msg.pose.orientation.w = (cos(latest_end_effector_pos_[3]*0.5*M_PI/180.0)); //assumes only changes in yaw
            cart_pos_msg.pose.orientation.x = 0;
            cart_pos_msg.pose.orientation.y = 0;
            cart_pos_msg.pose.orientation.z = sqrt(1-pow(cart_pos_msg.pose.orientation.w,2));

            cart_pos_msg.header.stamp = ros_time;
            joint_ang_msg.header.stamp = ros_time;
            _joint_state_pub.publish(joint_ang_msg);
            _end_effector_state_pub.publish(cart_pos_msg);
        }

        _rate.sleep();
    }
}

void DobotRosWrapper::robotControlLoop()
{

}

void DobotRosWrapper::JointTargetCallback(const std_msgs::Float64& msg)
{
    ROS_INFO("New target joint configurations received !!");
    
    target_joint_data_.mtx.lock();
    target_joint_data_.joint_data = msgs->data;   
    target_joint_data.mtx.unlock();

}

void DobotRosWrapper::EndEffectorTargetPoseCallback(const sensor_msgs::Pose& msg)
{
    ROS_INFO("New target end effector pose received !!");

    target_end_effector_pose_data_.mtx.lock();
    target_end_effector_pose_data_.pose_data = msgs->poses;
    target_end_effector_pose_data_.mtx.unlock();
}