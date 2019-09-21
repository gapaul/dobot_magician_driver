#ifndef DOBOT_DRIVER_H_
#define DOBOT_DRIVER_H_

#include "dobot_communication.h"
#include "dobot_state.h"


class DobotDriver{

private:
    DobotCommunication *_dobot_serial;
    DobotStates *_dobot_states;

    bool _is_e_stopped;
    bool _is_on_linear_rail;

public:


    DobotDriver(std::string port);

    bool getCurrentConfiguration(std::vector<double> &cart_pos, std::vector<double> &joint_angles);

    /**
     * @brief Specifies the correct instructions for the command packet to obtain the
     * current joint angles, and interprets the returned data from the Dobot
     * @param joint_angles: contains the current joint angles of the Dobot
     * @param bool indicates that the command was received by the Dobot
     */
    bool getJointAngles(std::vector<double> &joint_angles);

    /**
     * @brief Specifies the correct instructions for the command packet to obtain the
     * current end effector pose, and interprets the returned data from the Dobot
     * @param cart_pos: contains the desired positions in Cartesian space to be set
     * @param bool indicates that the command was received by the Dobot
     */
    bool getCartesianPos(std::vector<double> &cart_pos);

    /**
     * @brief Specifies the correct instructions for the command packet, to set the
     * joint angles of the Dobot, and interprets the returned data from the Dobot
     * @param joint_angles: contains the desired joint angles to be set
     * @param bool indicates that the command was received by the Dobot
     */
    bool setJointAngles(std::vector<float> &joint_angles);

    /**
     * @brief Specifies the correct instructions for the command packet, to set the
     * end effector pose, and interprets the returned data from the Dobot
     * @param cart_pos: contains the desired positions in Cartesian space to be set
     * @param bool indicates that the command was received by the Dobot
     */
    bool setCartesianPos(std::vector<float> &cart_pos);

    bool setCartesianPosWithRail(std::vector<float> &cart_pos);

    /**
     * @brief Specifies the correct instructions for the command packet to set the desired gripper
     * state, and interprets the returned data from the Dobot
     * @param is_ctrl_enabled: indicates whether the air pump controlling the gripper should be enabled
     * @param is_gripped: indicates whether the gripper is enabled
     * @param bool indicates that the command was received by the Dobot
     */
    bool setGripper(bool is_ctrl_enabled, bool is_gripped);

    /**
     * @brief Specifies the correct instructions for the command packet to set the desired suction cup
     * state, and interprets the returned data from the Dobot
     * @param is_ctrl_enabled: indicates whether the air pump controlling the suction cup should be enabled
     * @param is_sucked: indicates whether the suction cup is enabled
     * @param bool indicates that the command was received by the Dobot
     */
    bool setSuctionCup(bool is_ctrl_enabled, bool is_sucked);

    /**
     * @brief Runs the initialisation sequence for the Dobot
     */

	bool setCPParams(std::vector<float> &cp_params, bool real_time_track);
	/**
	* @brief Specifies the correct instructions for the command packet to set CP mode parameters
	* @param is_ctrl_enabled: indicates whether the air pump controlling the suction cup should be enabled
	* @param is_sucked: indicates whether the suction cup is enabled
	* @param bool indicates that the command was received by the Dobot
	*/

    bool getCPParams(std::vector<float> &cp_params, uint8_t &real_time_track);

	bool setCPCmd(std::vector<float> &cp_cmd, bool cp_mode);
	/**
	* @brief Specifies the correct instructions for the command packet execute the CP command
	* @param CPCmd: contains the desired parameters for the CP (x,y,z,velocity)
	* @param cpMode: indicates the CP mode (Relative or Absolute)
	* @param bool indicates that the command was received by the Dobot
	*/

    void initialiseDobot();

    bool setLinearRailStatus(bool is_enabled = false);

//    bool setHomeCalibrate()


    /*
     *  I/O COMMANDS
     */

    /*
     * @brief Function sends a command to the Dobot to set the multiplexing of its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param multiplex: the multiplexing
         0 - IOFunctionDummy;  // Invalid
         1 - IOFunctionDO;     // I/O output
         2 - IOFunctionPWM;    // PWM output
         3 - IOFunctionDI;     // I/O input
         4 - IOFunctionADC;    // A/D input
         5 - IOFunctionDIPU;   // Pull-up input
         6 - IOFunctionDIPD    // Pull-down input
     */
    bool setIOMultiplexing(int address, int multiplex);

    /*
     * @brief Function sends a command to the Dobot to get the multiplexing of its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param multiplex: the multiplexing we are trying to get
         0 - IOFunctionDummy;  // Invalid
         1 - IOFunctionDO;     // I/O output
         2 - IOFunctionPWM;    // PWM output
         3 - IOFunctionDI;     // I/O input
         4 - IOFunctionADC;    // A/D input
         5 - IOFunctionDIPU;   // Pull-up input
         6 - IOFunctionDIPD    // Pull-down input
     */
    bool getIOMultiplexing(int address, int &multiplex);

    /**
     * @brief Function sends a command to the Dobot to send a digital output to its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param level: the level output (0-LOW, 1-HIGH)
     */
    bool setIODigitalOutput(int address, bool level);

    /**
     * @brief Function sends a command to the Dobot to get the digital output of its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param level: the level output we are trying to get (0-LOW, 1-HIGH)
     */
    bool getIODigitalOutput(int address, bool &level);

    /**
     * @brief Function sends a command to the Dobot to send a PWM signal at an IO pin address
     * @param address: the address of the IO pin (from 1-20)
     * @param frequency: the PWM frequency (10Hz - 1MHz)
     * @param duty_cycle: the PWM duty ratio (0 - 100)
     */
    bool setIOPWM(int address, float frequency, float duty_cycle);

    /**
     * @brief Function sends a command to the Dobot to get the digital input of its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param level: the level input we are trying to get (0-LOW, 1-HIGH)
     */
    bool getIODigitalInput(int address, bool &level);

    /**
     * @brief Function sends a command to the Dobot to get the the 10-bit ADC value on its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param value: the 12-bit ADC value that we are reading (0-4095)
     */
    bool getIOAnalogInput(int address, int &value);

    /**
     * @brief Function turns the stepper motor on either the conveyor belt, linear rail or the   extruder stepper motor
     * @param index: sets which stepper port to control (0-stepper1 1-stepper2)
     * @param is_enabled: sets whether to turn on or turn off the stepper motor (0-0ff 1-on)
     * @param speed: sets the speed of the motor  (+ values clockwise, - values counterclockwise)
     */
    bool setEMotor(int index,bool is_enabled,int32_t speed);

    /**
     * @brief Function that starts the execution of queued commands in the buffer. This function must be called whenever 
     * a Stop of Force Stop function is called previously. If not, the Dobot will not be able to perform any subsequence 
     * actions. For Force Stop case, the Dobot will ignored the current command that was stopped by Force Stop and start 
     * executing the next one.
     * 
     * @return bool indicates whether the command was successful
     */
    bool setQueuedCmdStartExec(void);

    /**
     * @brief Function to stop the execution of all queued commands that are currently in the buffer. If this function is called
     * while the Dobot is performing an action, it will continue to finish that action and then stop. For example, if the Dobot 
     * is moving from point A to point B and this function is called, it will move to point B and stop.
     * Note: setQueuedCmdStartExec must be called to perform all of the remaining queued actions left in the buffer.
     * 
     * @return bool indicates whether the command was successful
     */
    bool setQueuedCmdStopExec(void);

    /**
     * @brief Function to IMMEDIATELY stop the execution of all queued commands that are currently in the buffer. If this function
     * is called while the Dobot is performing an action, it will stop immediately regardless what current action is. For example, 
     * if the Dobot is moving from point A to B and this function is called, it will stop immediately.
     * Note: setQueuedCmdStartExec must be called to perform all of the remaining queued actions left in the buffer. In this scenario,
     * the Dobot will ignore the current command that it is performing by the time the Force Stop is called and continue with the next
     * one in the buffer.
     * @return bool indicates whether the command was successful
     */
    bool setQueuedCmdForceStopExec(void);

    /**
     * @brief Function to clear the queued command buffer.
     * @return bool indicates whether the command was sucessfull
     */
    bool setQueuedCmdClear(void);

    /**
     * @brief Funtion to stop all IO ports (set them to be outputs of 0)
     * @return bool indicates whether the command was successfull
     */
    bool stopAllIO(void);

    /**
     * @brief Function to e-stop the Dobot.When executed, the Dobot will stop the current motion, the pump and all IO ports (1 to 20).
     * All of the queued command will also be cleared at the same time. The Dobot must be re-initalised to work normally.
     * @return bool indicates whether the command was successfull
     */
    bool setEStop(void);

    bool isEStopped(void);

    bool isOnLinearRail(void);

};

#endif /* DOBOT_DRIVER_H_ */
