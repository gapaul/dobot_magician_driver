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
#define JOINT_ERROR 0.1
#define LINEAR_ERROR 0.1
#define ORIENTATION_ERROR 0.01

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
    std::deque<sensor_msgs::JointState> joint_deq;
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
    sensor_msgs::JointState joint_data;
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
        // Publisher
        ros::Publisher joint_state_pub_;
        ros::Publisher end_effector_state_pub_;

        // Subscriber 
        ros::Subscriber target_joint_sub_;
        ros::Subscriber target_end_effector_sub_;

    private:
        
        bool robot_in_motion_;
        bool robot_at_target_;

        std::string port_;

        JointData target_joint_data_;
        PoseData target_end_effector_pose_data_;

        std::thread *update_state_thread_;
        std::thread *robot_control_thread_;

        std::vector<float> latest_joint_angles_;
        std::vector<float> latest_end_effector_pos_;

        void updateStateLoop();
        void robotControlLoop();

        void jointTargetCallback(const sensor_msgs::Pose& msg);
        void endEffectorTargetPoseCallback(const sensor_msgs::JointState& msg);
        
        bool moveToTargetJoints();
        bool moveToTargetEndEffectorPose();
};

#endif