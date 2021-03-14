#ifndef _DOBOT_UTILS_H_
#define _DOBOT_UTILS_H_

#include <cmath>
#include <deque>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

// General

#define MAX_BUFFER_SIZE 10
#define CONTROL_RATE 100

// dobot_communication.h

#define SERIAL_TIMEOUT 300 // miliseconds
#define TRY_LIMIT 100


// dobot_state.h

// Robot state

struct Pose
{
    double x;
    double y;
    double z;
    double theta;
};

struct JointConfiguration
{
    std::vector<double> position;
    std::vector<double> velocity;   
};

struct CartesianVelocity
{
    double x;
    double y;
    double z;
};

// IO State

// Dobot Params
// CP Params

struct ContinuousPathParams
{
    double plan_accel;
    double junction_velocity;
    union 
    {
        double actual_accel;
        double period;
    };
    bool real_time_track;
    std::atomic<bool> received;
    std::atomic<bool> user_set;
    std::mutex mtx;
};

// dobot_controlller.h

// dobot_driver.h

struct PoseBuffer
{
    std::deque<Pose> pose_data;
    std::mutex mtx;
    std::atomic<bool> received;
};

struct JointConfigurationBuffer
{
    std::deque<JointConfiguration> joint_data;
    std::mutex mtx;
    std::atomic<bool> received;
};

#endif