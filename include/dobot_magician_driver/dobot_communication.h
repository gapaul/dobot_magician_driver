#ifndef DOBOT_COMMUNICATION_H_
#define DOBOT_COMMUNICATION_H_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <cstring>


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
    uint64_t getPose(std::vector<u_int8_t> &returned_data, bool is_queued = 0);
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
    /**
     * @brief Function sends a command to the Dobot to set the status of the suction cup
     * @param is_ctrl_enabled: indicates whether the air pump controlling the suction cup should be enabled
     * @param is_sucked: indicates whether the suction cup is enabled
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, bool is_queued = 0);
    /**
     * @brief Function sends a command to the Dobot to get the status of the suction cup
     * @param returned_data: container that holds the "params" component of the payload from the returned
     * command packet
     * @param is_queued: indicates whether the instruction should be a queue command
     * @return uint64_t is the queue command index returned from the dobot if is_queued = 1
     */
    uint64_t getEndEffectorSuctionCup(std::vector<u_int8_t> &returned_data, bool is_queued = 0);
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
//    setCPParams
//    getCPParams
//    setPTPCmd

    /// ARC
//    setARCParams
//    getARCParams
//    setARCCmd

    /// WAIT
//    setWAITCmd

    /// TRIG
//    setTRIGCmd

    /// EIO
//    setIOMultiplexing
//    getIOMultiplexing
//    setIODO
//    getIODO
//    setIOPWM
//    getIOPWM
//    getIODI
//    getIOADC
    uint64_t setEMotor(int index,int insEnabled,float speed,bool is_queued);

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






};

#endif /* DOBOT_COMMUNICATION_H_ */
