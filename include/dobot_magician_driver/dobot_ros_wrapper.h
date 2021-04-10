#ifndef _DOBOT_ROS_WRAPPER_H_
#define _DOBOT_ROS_WRAPPER_H_

#include <cmath>
#include <deque>
#include <thread>

#include "ros/ros.h"
#include "ros/console.h"
#include "tf/tf.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include "dobot_driver.h"
#include "dobot_state.h"
#include "dobot_controller.h"
#include "dobot_communication.h"
#include "dobot_utils.h"

struct PoseDataBuffer
{
    std::deque<geometry_msgs::Pose> pose_deq;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct JointDataBuffer
{
    std::deque<sensor_msgs::JointState> joint_deq;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct PoseData
{
    geometry_msgs::Pose pose_data;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct JointData
{
    trajectory_msgs::JointTrajectoryPoint joint_data;
    std::mutex mtx;
    std::atomic<bool> received;
};

class DobotRosWrapper
{
    public:

        DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port);
        ~DobotRosWrapper();

        void init();
        void run();
        void stop();

    public:

        // Public ROS stuff

    private: 
        
        // Private Ros stuff
        // Node handle
        ros::NodeHandle nh_;
        ros::NodeHandle ph_;
        ros::Rate rate_;

        // Publisher
        ros::Publisher joint_state_pub_;
        ros::Publisher end_effector_state_pub_;

        ros::Publisher rail_position_pub_;

        ros::Publisher tool_state_pub_;

        ros::Publisher safety_state_pub_;

        ros::Publisher io_data_pub_;

        // Subscriber 
        ros::Subscriber target_joint_traj_sub_;
        ros::Subscriber target_end_effector_sub_;

        ros::Subscriber tool_state_sub_;

        ros::Subscriber safety_state_sub_;

        ros::Subscriber use_linear_rail_sub_;
        ros::Subscriber target_rail_sub_;

        ros::Subscriber e_motor_sub_;

        ros::Subscriber io_control_sub_;

        ros::Subscriber custom_command_sub_;

        // Callback functions
        void endEffectorTargetPoseCallback(const geometry_msgs::PoseConstPtr& msg);
        void jointTargetCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg);

        void toolStateCallback(const std_msgs::BoolConstPtr& msg);

        void safetyStateCallback(const std_msgs::UInt8ConstPtr& msg);

        void linearRailStateCallback(const std_msgs::BoolConstPtr& msg);

        void targetRailPositionCallback(const std_msgs::Float64ConstPtr& msg);
        
        void eMotorCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

        void ioStateCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

        // Use this at your own risk - for development and debugging only
        void customCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

    private:

        std::shared_ptr<DobotCommunication> dobot_serial_;
        std::shared_ptr<DobotStates> dobot_states_manager_;
        std::shared_ptr<DobotController> dobot_controller_;
        
        std::shared_ptr<DobotDriver> dobot_driver_;

        std::string port_;

        std::vector<std::string> joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"};

        JointData target_joint_data_;
        PoseData target_end_effector_pose_data_;

        std::atomic<bool> rail_position_received_;

        std::thread *update_state_thread_;
        std::thread *robot_control_thread_;

        std::mutex mtx_;
        std::vector<double> custom_command_;

        void updateStateThread();
        void robotControlThread();

        bool moveToTargetJoints();
        bool moveToTargetEndEffectorPose();
};

#endif