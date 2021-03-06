#ifndef _DOBOT_ROS_WRAPPER_H_
#define _DOBOT_ROS_WRAPPER_H_

#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "dobot_ros_wrapper_interface.h"

#define DOBOT_INIT_TIME 30
#define MAX_DATA_DEQUE_SIZE 10;

enum IOMux
{
    IODummy,  // Invalid
    IODO,     // I/O output
    IOPWM,    // PWM output
    IODI,     // I/O input
    IOADC,    // A/D input
    IODIPU,   // Pull-up input
    IODIPD    // Pull-down input
};

struct PoseDataBuffer
{
    std::deque<sensor_msgs::Pose> pose_deq;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct JointDataBuffer
{
    std::deque<std_msgs::Float64> joint_deq;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct PoseData
{
    sensor_msgs::Pose pose_data;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct JointData
{
    std_msgs::Float64 joint_data;
    std::mutex mtx;
    std::atomic<bool> received;
}

class DobotRosWrapper : public DobotRosWrapperInterface
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

    private:
        
        JointData target_joint_data_;
        PoseData target_end_effector_pose_data_;

        std::thread *update_state_thread_;
        std::thread *robot_control_thread_;

        std::vector<double> latest_joint_angles_;
        std::vector<double> latest_end_effector_pos_;

        void updateStateLoop();
        void robotControlLoop();

        void JointTargetCallback(const sensor_msgs::Pose& msg);
        void EndEffectorTargetPoseCallback(const std_msgs::Float64& msg);
        
};

#endif