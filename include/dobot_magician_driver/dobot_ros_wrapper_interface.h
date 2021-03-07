#ifndef _DOBOT_ROS_WRAPPER_INTERFACE_H_
#define _DOBOT_ROS_WRAPPER_INTERFACE_H_

#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "dobot_driver.h"

class DobotRosWrapperInterface
{
    public:
    
        DobotRosWrapperInterface();
        ~DobotRosWrapperInterface();

        virtual void init() = 0;
        virtual void run() = 0;
        
        void setDriver(DobotDriver *driver);
        DobotDriver getDriver(); 

    protected:
        
        DobotDriver *driver_;

        ros::NodeHandle nh_;
        ros::NodeHandle ph_;
        ros::Rate rate_;
};  

#endif 