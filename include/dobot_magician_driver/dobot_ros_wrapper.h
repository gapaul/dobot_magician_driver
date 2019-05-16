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

    /**
     * @brief ROS Service to set the status of the gripper
     * @param req: contains the desired parameters to be set
     * @param res: contains whether the command was received by the Dobot
     * @param bool indicates whether invalid parameters were set
     */
    bool setGripper(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res);
    /**
     * @brief ROS Service to set the status of the suction cup
     * @param req: contains the desired parameters to be set
     * @param res: contains whether the command was received by the Dobot
     * @param bool indicates whether invalid parameters were set
     */
    bool setSuctionCup(dobot_magician_driver::SetEndEffectorRequest &req, dobot_magician_driver::SetEndEffectorResponse &res);
    /**
     * @brief ROS Service to command the joint angles to a desired position
     * @param req: contains the desired joint angles to be set in degrees
     * @param res: contains whether the command was received by the Dobot
     * @param bool indicates whether invalid parameters were set
     */
    bool setJointAngles(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res);
    /**
     * @brief ROS Service to command the end effector to a desired pose
     * @param req: contains the desired pose specified in millimetres
     * @param res: contains whether the command was received by the Dobot
     * @param bool indicates whether invalid parameters were set
     */
    bool setCartesianPos(dobot_magician_driver::SetTargetPointsRequest &req, dobot_magician_driver::SetTargetPointsResponse &res);

    std::thread *update_state_thread;
    /**
     * @brief Publishes "state" of the Dobot
     */
    void update_state_loop();

public:
    DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port);
    ~DobotRosWrapper();
};


#endif /* DOBOT_ROS_WRAPPER_H_ */
