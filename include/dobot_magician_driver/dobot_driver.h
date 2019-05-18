#ifndef DOBOT_DRIVER_H_
#define DOBOT_DRIVER_H_

#include "dobot_communication.h"
#include "dobot_state.h"


class DobotDriver{

private:
    DobotCommunication *_dobot_serial;
    DobotStates *_dobot_states;

public:


    DobotDriver(std::string port);

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

	bool setCPParams(std::vector<float> &CPParams, int realtimeTrack, bool is_queued);
	/**
	* @brief Specifies the correct instructions for the command packet to set CP mode parameters
	* @param is_ctrl_enabled: indicates whether the air pump controlling the suction cup should be enabled
	* @param is_sucked: indicates whether the suction cup is enabled
	* @param bool indicates that the command was received by the Dobot
	*/

	bool setCPCmd(std::vector<float> &CPCmd, int cpMode, bool is_queued);
	/**
	* @brief Specifies the correct instructions for the command packet execute the CP command
	* @param CPCmd: contains the desired parameters for the CP (x,y,z,velocity)
	* @param cpMode: indicates the CP mode (Relative or Absolute)
	* @param bool indicates that the command was received by the Dobot
	*/

    void initialiseDobot(void);

//    bool setHomeCalibrate()


    /*
     *  I/O COMMANDS
     */

    /*
     * @brief Function sends a command to the Dobot to set the multiplexing of its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param multiplex: the multiplexing
          IOFunctionDummy;  // Invalid
          IOFunctionDO;     // I/O output
          IOFunctionPWM;    // PWM output
          IOFunctionDI;     // I/O input
          IOFunctionADC;    // A/D input
          IOFunctionDIPU;   // Pull-up input
          IOFunctionDIPD    // Pull-down input
     */
    bool setIOMUX(uint8_t address, uint8_t multiplex);

};

#endif /* DOBOT_DRIVER_H_ */
