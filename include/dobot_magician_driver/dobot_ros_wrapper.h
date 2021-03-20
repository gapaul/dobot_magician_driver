#ifndef _DOBOT_ROS_WRAPPER_H_
#define _DOBOT_ROS_WRAPPER_H_

#include <cmath>
#include <deque>
#include <thread>

#include "ros/ros.h"
#include "ros/console.h"
#include "tf/tf.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

#include "dobot_driver.h"
#include "dobot_state.h"
#include "dobot_controller.h"
#include "dobot_communication.h"
#include "dobot_utils.h"

#define DOBOT_INIT_TIME 30
#define MAX_DATA_DEQUE_SIZE 10
// #define JOINT_ERROR 0.1
// #define LINEAR_ERROR 0.1
// #define ORIENTATION_ERROR 0.01

#define IO_PIN_MIN 1
#define IO_PIN_MAX 20
#define IO_PWM_HZ_MIN 10        // Hz
#define IO_PWM_HZ_MAX 1000000   // Hz
#define IO_PWM_DC_MIN 0     // %
#define IO_PWM_DC_MAX 100   // %

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
    sensor_msgs::JointState joint_data;
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

        // Subscriber 
        ros::Subscriber target_joint_sub_;
        ros::Subscriber target_end_effector_sub_;

    private:

        std::shared_ptr<DobotCommunication> dobot_serial_;
        std::shared_ptr<DobotStates> dobot_states_manager_;
        std::shared_ptr<DobotController> dobot_controller_;
        
        std::unique_ptr<DobotDriver> dobot_driver_;

        std::string port_;

        JointData target_joint_data_;
        PoseData target_end_effector_pose_data_;

        std::thread *update_state_thread_;
        std::thread *robot_control_thread_;

        void updateStateThread();
        void robotControlThread();

        void endEffectorTargetPoseCallback(const geometry_msgs::PoseConstPtr& msg);
        void jointTargetCallback(const sensor_msgs::JointStateConstPtr& msg);
        
        void setTargetEndEffectorPose();
        void setTargetJointConfiguration();

        bool moveToTargetJoints();
        bool moveToTargetEndEffectorPose();
};

#endif