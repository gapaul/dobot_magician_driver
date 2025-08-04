#ifndef _DOBOT_ROS_WRAPPER_H_
#define _DOBOT_ROS_WRAPPER_H_

#include <cmath>
#include <deque>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "dobot_driver.h"
#include "dobot_state.h"
#include "dobot_controller.h"
#include "dobot_communication.h"
#include "dobot_utils.h"

struct PoseDataBuffer
{
    std::deque<geometry_msgs::msg::Pose> pose_deq;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct JointDataBuffer
{
    std::deque<sensor_msgs::msg::JointState> joint_deq;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct PoseData
{
    geometry_msgs::msg::Pose pose_data;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct JointData
{
    trajectory_msgs::msg::JointTrajectoryPoint joint_data;
    std::mutex mtx;
    std::atomic<bool> received;
};

class DobotRosWrapper : public rclcpp::Node
{
    public:

        DobotRosWrapper(std::string node_name, std::string port);
        ~DobotRosWrapper();

        void init();
        void run();
        void stop();

    public:

        // Public ROS stuff

    private: 
        
        // Private Ros stuff
        rclcpp::Rate rate_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr end_effector_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rail_position_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tool_state_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr safety_state_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr io_data_pub_;

        // Subscribers 
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr target_joint_traj_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_end_effector_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tool_state_sub_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr safety_state_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr use_linear_rail_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_rail_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr e_motor_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr io_control_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr custom_command_sub_;

        // Callback functions
        void endEffectorTargetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void jointTargetCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
        void toolStateCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
        void safetyStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);
        void linearRailStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void targetRailPositionCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void eMotorCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void ioStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        // Use this at your own risk - for development and debugging only
        void customCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

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