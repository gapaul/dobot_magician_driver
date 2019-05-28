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
#include "dobot_magician_driver/SetEMotor.h"

#include "dobot_magician_driver/SetIOMultiplexing.h"
#include "dobot_magician_driver/SetIODigitalOutput.h"
#include "dobot_magician_driver/SetIOPWMOutput.h"
#include "dobot_magician_driver/GetIODigitalInput.h"
#include "dobot_magician_driver/GetIOAnalogInput.h"

#include "dobot_magician_driver/SetCPParams.h"
#include "dobot_magician_driver/SetCPCmd.h"


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
    ros::ServiceServer _set_eMotor_srv;

    ros::ServiceServer _set_io_multiplex_srv;
    ros::ServiceServer _set_io_digital_output_srv;
    ros::ServiceServer _set_io_pwm_output_srv;
    ros::ServiceServer _get_io_digital_input_srv;
    ros::ServiceServer _get_io_analog_input_srv;

    ros::ServiceServer _set_cp_params_srv;
    ros::ServiceServer _set_cp_cmd_srv;

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

    /*
     *  I/O
     */

    /**
     * @brief ROS Service to set the multiplexing of IO pins
     * @param req: contains the address of a pin (1-20), and the multiplexing value (0-6)
         0 - IOFunctionDummy;  // Invalid
         1 - IOFunctionDO;     // I/O output
         2 - IOFunctionPWM;    // PWM output
         3 - IOFunctionDI;     // I/O input
         4 - IOFunctionADC;    // A/D input
         5 - IOFunctionDIPU;   // Pull-up input
         6 - IOFunctionDIPD    // Pull-down input
     * @param res: contains whether the command was received by the Dobot
     * @return bool indicates whether invalid parameters were set
     */
    bool setIOMultiplexing(dobot_magician_driver::SetIOMultiplexingRequest &req, dobot_magician_driver::SetIOMultiplexingResponse &res);

    /**
     * @brief ROS Service to set the digital output level of IO pins
     * @param req: contains the address of a pin (1-20), and the output level (0: LOW and 1: HIGH)
     * @param res: contains whether the command was received by the Dobot
     * @return bool indicates whether invalid parameters were set
     */
    bool setIODigitalOutput(dobot_magician_driver::SetIODigitalOutputRequest &req, dobot_magician_driver::SetIODigitalOutputResponse &res);

    /**
     * @brief ROS Service to set the Pulse Width Modulation (PWM) output of IO pins
     * @param req: contains the address of a pin (1-20), the PWM frequency (10Hz - 1MHz) and its duty cycle (0-100)
     * @param res: contains whether the command was received by the Dobot
     * @return bool indicates whether invalid parameters were set
     */
    bool setIOPWMOutput(dobot_magician_driver::SetIOPWMOutputRequest &req, dobot_magician_driver::SetIOPWMOutputResponse &res);

    /**
     * @brief ROS Service to get the digital input level of IO pins
     * @param req: contains the address of a pin (1-20)
     * @param res: contains the level we are reading and whether the command was received by the Dobot
     * @return bool indicates whether invalid parameters were set
     */
    bool getIODigitalInput(dobot_magician_driver::GetIODigitalInputRequest &req, dobot_magician_driver::GetIODigitalInputResponse &res);

    /**
     * @brief ROS Service to get the analog input value (ADC value) of IO pins
     * @param req: contains the address of a pin (1-20)
     * @param res: contains the ADC value we are reading and whether the command was received by the Dobot
     * @return bool indicates whether invalid parameters were set
     */
    bool getIOAnalogInput(dobot_magician_driver::GetIOAnalogInputRequest &req, dobot_magician_driver::GetIOAnalogInputResponse &res);

    bool setEMotor(dobot_magician_driver::SetEMotorRequest &req, dobot_magician_driver::SetEMotorResponse &res);

    /**
     * @brief ROS Service to set the Continuos Path Parameters
     * @param req: contains the CP parameters
     * @param res: contains whether the command was received by the Dobot
     * @return bool indicates whether invalid parameters were set
     */
    bool setCPParams(dobot_magician_driver::SetCPParamsRequest &req, dobot_magician_driver::SetCPParamsResponse &res);

    bool setCPCmd(dobot_magician_driver::SetCPCmdRequest &req, dobot_magician_driver::SetCPCmdResponse &res);

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
