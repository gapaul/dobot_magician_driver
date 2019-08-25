#ifndef DOBOT_COMMUNICATION_H_
#define DOBOT_COMMUNICATION_H_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <cstring>
#include <limits>
#include <cmath>


#include <SerialPort.h>
#include <SerialStream.h>
#include <SerialStreamBuf.h>


class DobotCommunication{

public:
    DobotCommunication(std::string port);
    ~DobotCommunication();

    /// Device information
//    setDeviceSN
//    getDeviceSN
//    setDeviceName
//    setDeviceName
//    getDeviceVersion

    /// Pose
    /**
     * @brief Function sends a command to the Dobot to obtain the real-time "pose" (as defined in the
     * communication protocol) of the Dobot
     * @param returned_data: container that holds the "params" component of the payload from the returned
     * command packet
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    bool getPose(std::vector<u_int8_t> &returned_data);
//    bool resetPose();

    /// Alarm
//    bool getAlarmState();
//    bool clearAlarmState();

    /// Home
//    setHOMEParams
//    getHOMEParams
    /**
     * @brief Function sends a command to the Dobot to execute the homing function
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t setHOMECmd(bool is_queued = 1);

    /// Handheld teaching
//    setHHrigMode
//    getHHrigMode
//    setHHrigOutputEnabled
//    getHHrigOutputEnabled
//    getHHTTrigOutput

    /// Arm orientation
//    getArmOrientation
//    setArmOrientation

    /// EndEffector
//    setEndEffectorParams
//    getEndEffectorParams
//    setEndEffectorLaser
//    getEndEffectorLaser
    bool setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, bool is_queued = 0);
    /**
     * @brief Function sends a command to the Dobot to set the status of the suction cup
     * @param is_ctrl_enabled: indicates whether the air pump controlling the suction cup should be enabled
     * @param is_sucked: indicates whether the suction cup is enabled
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    bool setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, uint64_t &queue_cmd_index, bool is_queued = 1);
    /**
     * @brief Function sends a command to the Dobot to get the status of the suction cup
     * @param returned_data: container that holds the "params" component of the payload from the returned
     * command packet
     * @return bool indicates whether the command sent was successful
     */
    bool getEndEffectorSuctionCup(std::vector<u_int8_t> &returned_data);
    /**
     * @brief Function sends a command to the Dobot to set the status of the gripper
     * @param is_ctrl_enabled: indicates whether the air pump controlling the gripper should be enabled
     * @param is_gripped: indicates whether the gripper is enabled
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t setEndEffectorGripper(bool is_ctrl_enabled, bool is_gripped, bool is_queued = 0);
//    getEndEffectorGripper

    /// JOG
//    setJOGJointParams
//    getJOGJointParams
//    setJOGCoordinateParams
//    getJOGCoordinateParams
//    setJOGCommonParams
//    getJOGCommonParams
//    setJOGCmd

    /// PTP
//    setPTPJointParams
//    getPTPJointParams
//    setPTPCoordinateParams
//    getPTPCoordinateParams
//    setPTPJumpParams
//    getPTPJumpParams
//    setPTPCommonParams
//    getPTPCommonParams
    /**
     * @brief Function sends a command to the Dobot to execute the specified PTP command
     * @param ptp_mode: indicates the PTP mode desired
     * @param target_points: parameters to set in the PTP command (joint angles, angle increments, point)
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t setPTPCmd(int ptp_mode, std::vector<float> &target_points, bool is_queued = 1);

    ///CP
	/**
	* @CP: Continuos Path
	* @CP parameters:
	*	*planAcc: Maximum planned acceleration
	*	*junctionVel: Maximum junction acceleration
	*	*acc: Maximum actual acceleration,used in non-real-time mode only
	*	*period: Interpolation cycle, used in real-time mode only
	*	*realtimeTrack: 0: Non-real time mode; 1: Real time mode
	*/
	bool setCPParams(std::vector<float> &cp_params, bool real_time_track, bool is_queued);
    bool setCPParams(std::vector<float> &cp_params, bool real_time_track, uint64_t &queue_cmd_index, bool is_queued);
	/**
	 * @brief Function sends a command to the Dobot to set Parameters for the CP mode
	 * @param cp_params: Parameters for CP mode
	 * @param real_time_track: parameter to indicate CP mode (0: non-real time mode, 1: real time mode)
	 * @param is_queued: indicates whether the instruction should be a queue command
	 * @param queue_cmd_index is the queue command index returned from the dobot if is_queued = 1
     * @return bool indicates whether the command sent was successful
	 */
	bool getCPParams(std::vector<u_int8_t> &returned_data);
	/**
	 * @brief Function sends a command to the Dobot to get the CP Parameters
	 * @param returned_data: Parameters for CP mode
	 * @param is_queued: indicates whether the instruction should be a queue command
	 * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
	 */
    bool setCPCmd(std::vector<float> &cp_cmd, bool cp_mode, bool is_queued);
	bool setCPCmd(std::vector<float> &cp_cmd, bool cp_mode, uint64_t &queue_cmd_index, bool is_queued);
	/**
	 * @brief Function sends a command to the Dobot to execute the specific CP Command
	 * @param CPCmd: CP parameters (x, y, z, velocity)
	 * @param cpMode: CP mode (0: Relative - Cartesian Increment, 1: Absolute - Cartesian Coordinate)
	 * @param is_queued: indicates whether the instruction should be a queue command
	 * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
	 */
    /// ARC
//    setARCParams
//    getARCParams
//    setARCCmd

    /// WAIT
//    setWAITCmd

    /// TRIG
//    setTRIGCmd

    /// EIO

    /**
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
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t setIOMultiplexing(int address, int multiplex, bool is_queued = 0);

    /**
     * @brief Function sends a command to the Dobot to get the multiplexing of IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param returned_data: container that holds the "params" component of the payload from the returned
     * command packet
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     * Here it will always return -1 because the command is not queued by default
     */
    uint64_t getIOMultiplexing(int address, std::vector<u_int8_t> &returned_data);

    /**
     * @brief Function sends a command to the Dobot to send a digital output on its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param level: the level output (0-LOW, 1-HIGH)
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t setIODO(int address, bool level, bool is_queued = 0);

    /**
     * @brief Function sends a command to the Dobot to get the digital output on its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param returned_data: container that holds the "params" component of the payload from the returned
     * command packet
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     * Here it will always return -1 because the command is not queued by default
     */
    uint64_t getIODO(int address, std::vector<u_int8_t> &returned_data);

    /**
     * @brief Function sends a command to the Dobot to send a PWM signal at an IO pin address
     * @param address: the address of the IO pin (from 1-20)
     * @param frequency: the PWM frequency (10Hz - 1MHz)
     * @param duty_cycle: the PWM duty ratio (0 - 100)
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t setIOPWM(int address, float frequency, float duty_cycle, bool is_queued = 0);

//    getIOPWM

    /**
     * @brief Function sends a command to the Dobot to get the digital input on its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param returned_data: container that holds the "params" component of the payload from the returned
     * command packet
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     * Here it will always return -1 because the command is not queued by default
     */
    uint64_t getIODI(int address, std::vector<u_int8_t> &returned_data);

    /**
     * @brief Function sends a command to the Dobot to get the the 10-bit ADC value on its IO pins
     * @param address: the address of the IO pin (from 1-20)
     * @param returned_data: container that holds the "params" component of the payload from the returned
     * command packet
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     * Here it will always return -1 because the command is not queued by default
     */
    uint64_t getIOADC(int address, std::vector<u_int8_t> &returned_data);

    uint64_t setEMotor(int index,bool is_enabled,float speed,bool direction,bool is_queued = 0);
    /**
     * @brief Function controls stepper motor for conveyor belt, linear rail or extruder motor
     * @param address: the address of the IO pin (from 1-20)
     * @param index: selects between stepper 1 or stepper 2 port
     * @param is_enabled:turns on/off the stepper motor for selected port
     * @param speed:changes the velocity (pulses/sec) and direction of stepper motor (+ values clockwise, - values counterclockwise)
     * @param is_queued: indicates whether the instruction should be a queue command
     * Here it will always return -1 because the command is not queued by default
     */

    /// Calibration
//    setAngleSensorStaticError
//    getAngleSensorStaticError

    /// WIFI
//    setWIFIConfiMode
//    getWIFIConfigMode
//    setWIFISSID
//    getWIFISSID
//    setWIFIPassword
//    getWIFIPassword
//    setWIFIIPAddress
//    getWIFIIPAddress
//    setWIFINetMask
//    getWIFINetMask
//    setWIFIGateway
//    getWIFIGateway
//    setWIFIDNS
//    getWIFIDNS
//    getWIFIConnectStatus

    /// Queued execution control commands
//    SetQueuedCmdStartExec
//    SetQueuedCmdStopExec
//    SetQueuedCmdForceStopExec
//    SetQueuedCmdStartDownload
//    SetQueuedCmdStopDownload
//    SetQueuedCmdClear
//    GetQueuedCmdCurrentIndex

    /**
     * @brief Function converts 4 uin8_t into a float
     * @param it: indicates the first of the four bytes to be converted, where the second, third
     * and fourth bytes are sequential
     * @return float is converted 4 uint_t values
     */
    float unpackFloatLE(std::vector<u_int8_t>::iterator it);

private:

    std::string _port;

    SerialPort::BaudRate _baud;
    SerialPort::Parity _parity;
    SerialPort::StopBits _stop_bit;
    SerialPort::CharacterSize _character_size;

    SerialPort* _serial_port;

    double _serial_timeout;
    int _try_limit;
    std::mutex _communication_mt;

    /**
     * @brief Based on the desired command packet, a checksum value is returned to ensure the correct
     * communication
     * @param ctrl_cmd: holds the desired command packet
     * @param u_int8_t checksum value based upon the instructions in the command packet
     */
    u_int8_t checksumCalc(std::vector<uint8_t> &ctrl_cmd);
    /**
     * @brief Function sends a command to the Dobot
     * @param ctrl_cmd: command packet to be sent
     */
    void sendCommand(std::vector<u_int8_t> &ctrl_cmd);
    /**
     * @brief Function once a command has been sent, this function is used to store the returned response
     * from the dobot in a vector.
     * @param returned_data: holds the returned response from the Dobot
     * @param bool indicates whether data was received
     */
    bool getResponse(std::vector<u_int8_t> &returned_data);
    /**
     * @brief Given a vector of floats, this function returns a corresponding vector of u_int8_t where
     * elements 0-3 indicate the first element in the float vector, 4-7 represent the second element in the
     * float vector
     * @param value_to_pack: contains the floats to be converted
     * @param packed_floats: contains the floats represented as 4 u_int8_t
     */
    void packFromFloat(std::vector<float> &value_to_pack, std::vector<u_int8_t> &packed_floats);
    /**
     * @brief Function to convert a float to it's corresponding u_int8_t representation (4 elements)
     * @param float: value to be converted
     * @param temp_bytes: converted value stored as a 4 element u_in8_t array
     */
    void floatToByte(float float_variable, u_int8_t temp_bytes[]);
    /**
     * @brief Given the returned response from the Dobot stored in data, the function returns the data
     * as a u_int64_t
     * @param data: holds the returned response from the Dobot
     * @param u_int64_t corresponding representation of the returned data
     */
    uint64_t getQueuedCmdIndex(std::vector<u_int8_t> data);
    /**
     * @brief Function returns boolean indicating whether data was read. If true, the variable next_char
     * is populated
     * @param next_char: contains the data read from the serial port
     * @return bool: indicates whether data was read from the port
     */
    bool tryReadByte(uint8_t &next_char);
};

#endif /* DOBOT_COMMUNICATION_H_ */
