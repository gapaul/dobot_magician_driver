#ifndef _DOBOT_ROS_WRAPPER_INTERFACE_H_
#define _DOBOT_ROS_WRAPPER_INTERFACE_H_

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

        ros::Publisher joint_state_pub_;
        ros::Publisher end_effector_state_pub_; //is this the best name?
        ros::Publisher tool_vel_pub_; //not sure if exists
};  

#endif 