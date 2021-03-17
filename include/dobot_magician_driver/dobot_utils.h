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

// dobot_communication.h //

#define SERIAL_TIMEOUT 300 // miliseconds
#define TRY_LIMIT 100

struct DataPacket
{

};

// dobot_state.h

// Robot state

struct Pose
{
    double x;
    double y;
    double z;
    double theta;

    bool operator==(Pose &input)
    {
        if(x != input.x) return false;
        if(y != input.y) return false;
        if(z != input.z) return false;
        if(theta != input.theta) return false;
        return true;
    }

    Pose operator+(Pose &input)
    {
        Pose output;
        output.x = x + input.x;
        output.y = y + input.y;
        output.z = z + input.z;
        output.theta = theta + input.theta;
        return output;
    }

    Pose operator-(Pose &input)
    {
        Pose output;
        output.x = x - input.x;
        output.y = y - input.y;
        output.z = z - input.z;
        output.theta = theta - input.theta;
        return output;
    }

    bool operator<(Pose &input)
    {
        if(x >= input.x) return false;
        if(y >= input.y) return false;
        if(z >= input.z) return false;
        if(theta >= input.theta) return false;
        return true;
    }

    bool operator<=(Pose &input)
    {
        if(x > input.x) return false;
        if(y > input.y) return false;
        if(z > input.z) return false;
        if(theta > input.theta) return false;
        return true;
    }

    bool operator>(Pose &input)
    {
        if(x <= input.x) return false;
        if(y <= input.y) return false;
        if(z <= input.z) return false;
        if(theta <= input.theta) return false;
        return true;
    }

    bool operator>=(Pose &input)
    {
        if(x < input.x) return false;
        if(y < input.y) return false;
        if(z < input.z) return false;
        if(theta < input.theta) return false;
        return true;
    }

    void convert(std::vector<double> &input)
    {
        x = input.at(0);
        y = input.at(1);
        z = input.at(2);
        theta = input.at(3);
    }

};

struct JointConfiguration
{
    std::vector<double> position;
    std::vector<double> velocity;   

    bool operator==(JointConfiguration &input)
    {
        for(int i = 0; i < position.size(); i++)
        {
            if(position.at(i) != input.position.at(i))
            {
                return false;
            }
        }
        return true;
    }
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

#define LINEAR_ERROR 1
#define ANGULAR_ERROR 1
#define JOINT_ERROR 0.5

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