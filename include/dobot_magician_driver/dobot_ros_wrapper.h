#ifndef DOBOT_ROS_WRAPPER_H_
#define DOBOT_ROS_WRAPPER_H_

#include <cmath>
#include <thread>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "dobot_driver.h"
#include "dobot_magician_driver/dobot_communication.h"
#include "dobot_magician_driver/SetEndEffector.h"
#include "dobot_magician_driver/SetTargetPoints.h"


class DobotRosWrapper {

private:
    DobotDriver *_driver;

    ros::NodeHandle _nh;
    ros::NodeHandle _pn;
    ros::Rate _rate;

    ros::Publisher _joint_state_pub;
    ros::Publisher _end_effector_state_pub; //is this the best name?
    ros::Publisher _tool_vel_pub; //not sure if exists


    ros::ServiceServer _set_gripper_srv;
    ros::ServiceServer _set_suction_cup_srv;
    ros::ServiceServer _set_cartesian_pos_srv;
    ros::ServiceServer _set_joint_angles_srv;

    bool setGripper(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res);
    bool setSuctionCup(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res);
    bool setJointAngles(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res);
    bool setCartesianPos(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res);

    std::thread *update_state_thread;
    void update_state_loop();

public:
    DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port);
    ~DobotRosWrapper();
};


#endif /* DOBOT_ROS_WRAPPER_H_ */
