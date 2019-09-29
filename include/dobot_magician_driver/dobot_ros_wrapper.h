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
#include "dobot_magician_driver/GetCPParams.h"
#include "dobot_magician_driver/SetCPCmd.h"

#include "dobot_magician_driver/SetQueuedCmd.h"

#include "dobot_magician_driver/SetEStop.h"

#include "dobot_magician_driver/SetInitialise.h"

#include "dobot_magician_driver/SetLinearRail.h"
#include "dobot_magician_driver/SetTargetPointsWithRail.h"

#define IO_PIN_MIN 1
#define IO_PIN_MAX 20
#define IO_PWM_HZ_MIN 10        // Hz
#define IO_PWM_HZ_MAX 1000000   // Hz
#define IO_PWM_DC_MIN 0     // %
#define IO_PWM_DC_MAX 100   // %

#define CP_PARAM_SIZE 3
#define CP_CMD_SIZE 4

enum IOMux {
    IODummy,  // Invalid
    IODO,     // I/O output
    IOPWM,    // PWM output
    IODI,     // I/O input
    IOADC,    // A/D input
    IODIPU,   // Pull-up input
    IODIPD    // Pull-down input
};

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
    ros::ServiceServer _get_cp_params_srv;
    ros::ServiceServer _set_cp_cmd_srv;

    ros::ServiceServer _set_queued_cmd_start_srv;
    ros::ServiceServer _set_queued_cmd_stop_srv;
    ros::ServiceServer _set_queued_cmd_force_stop_srv;
    ros::ServiceServer _set_queued_cmd_clear_srv;

    ros::ServiceServer _set_e_stop_srv;

    ros::ServiceServer _set_initialise_srv;

    ros::ServiceServer _set_linear_rail_srv;
    ros::ServiceServer _set_cartesian_pos_with_rail_srv;

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

    /*
     * CONTINUOUS PATH COMMAND
     */

    /**
     * @brief ROS Service to set the Continuos Path Parameters
     * @param req: contains the CP parameters
     * @param res: contains whether the command was received by the Dobot
     * @return bool indicates whether invalid parameters were set
     */
    bool setCPParams(dobot_magician_driver::SetCPParamsRequest &req, dobot_magician_driver::SetCPParamsResponse &res);

    /**
     * @brief ROS Service to get the current Conitnuous Path Paramemters from the Dobot
     * @param res: contains the CP Parameters
     * @return bool indicates whether the command was successfull
     */
    bool getCPParams(dobot_magician_driver::GetCPParamsRequest &req, dobot_magician_driver::GetCPParamsResponse &res);

    /**
     * @brief ROS Service to set the Continuous Path command for the Dobot
     * @param req: contains the CP command: coordinate points (x,y,z) and velocity, and CP mode: relative or absolute 
     * @param res: contains whether the command was received by the Dobot
     * @return bool indicates whether the command was succesfull
     */
    bool setCPCmd(dobot_magician_driver::SetCPCmdRequest &req, dobot_magician_driver::SetCPCmdResponse &res);

    /*
     * QUEUED COMMANDS CONTROL COMMAND
     */

    /**
     * @brief ROS Service to start executing all queued commands that are currently in the buffer now. This service must
     * be called whenever a Stop or Force_Stop Service is called previously or else the Dobot would not be able to perform 
     * any subsequence actions. For Force Stop case, the Dobot will ignored the current command that was stopped by Force Stop
     * Service and start executing the next one.
     * 
     * @return bool indicates whether the command was successfull or not
     */
    bool setQueuedCmdStartExec(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res);
    
    /**
     * @brief ROS Service to stop the execution of all queued commands that are currently in the buffer. If this Service is called
     * while the Dobot is performing an action, it will continue to finish that action and then stop. For example, if the Dobot 
     * is moving from point A to point B and this Service is called, it will move to point B and stop any subsequence action
     * Note: setQueuedCmdStartExec must be called to perform all of the remaining queued actions left in the buffer.
     * 
     * @return bool indicates whether the command was successfull or not
     */
    bool setQueuedCmdStopExec(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res);

    /**
     * @brief ROS Service to immediately stop the execution of all queued commands that are currently in the buffer. If this Service
     * is called while the Dobot is performing an action, it will stop regardless what current action is. For example, if the Dobot is 
     * moving from point A to B and this Service is called, it will stop immediately.
     * Note: setQueuedCmdStartExec must be called to perform all of the remaining queued actions left in the buffer. In this scenario,
     * the Dobot will ignore the current command that it is performing by the time the Force Stop is called and continue with the next
     * one in the buffer.
     * 
     * @return bool indicates whether the command was successfull or not
     */
    bool setQueuedCmdForceStopExec(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res);

    bool setQueuedCmdClear(dobot_magician_driver::SetQueuedCmdRequest &req, dobot_magician_driver::SetQueuedCmdResponse &res);

    /**
     * @brief ROS Service to e-stop the Dobot. When executed, the Dobot will stop the current motion, the pump and all IO ports (1 to 20).
     * All of the queued command will also be cleared at the same time. The Dobot must be re-initalised to work normally.
     * @return bool indicates whether the command was successfull or not
     */
    bool setEStop(dobot_magician_driver::SetEStopRequest &req, dobot_magician_driver::SetEStopResponse &res);
    
    /**
     * @brief ROS Service to initialise the Dobot. This includes the Homing procedure and it also clears the queued commands buffer and 
     * the IO port at the same time.
     */
    bool setInitialise(dobot_magician_driver::SetInitialiseRequest &req, dobot_magician_driver::SetInitialiseResponse &res);

    bool setLinearRailStatus(dobot_magician_driver::SetLinearRailRequest &req, dobot_magician_driver::SetLinearRailResponse &res);

    bool setCartesianPosWithRail(dobot_magician_driver::SetTargetPointsWithRailRequest &req, dobot_magician_driver::SetTargetPointsWithRailResponse &res);

    std::thread *update_state_thread;
    /**
     * @brief Publishes "state" of the Dobot
     */
    void update_state_loop();

    /**
     * @brief Checks if the pin address is within the IO range
     */
    bool inIORange(int address);

public:
    DobotRosWrapper(ros::NodeHandle &nh, ros::NodeHandle &pn, std::string port);
    ~DobotRosWrapper();
};


#endif /* DOBOT_ROS_WRAPPER_H_ */
