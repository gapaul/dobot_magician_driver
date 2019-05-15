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
    uint64_t getPose(std::vector<u_int8_t> &returned_data, bool isQueued = 0);
//    bool resetPose();

    /// Alarm
//    bool getAlarmState();
//    bool clearAlarmState();

    /// Home
//    setHOMEParams
//    getHOMEParams
    uint64_t setHOMECmd(bool isQueued = 1);

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
    uint64_t setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, bool isQueued = 0);
    uint64_t getEndEffectorSuctionCup(std::vector<u_int8_t> &returned_data, bool isQueued = 0);
//    setEndEffectorGripper
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
    uint64_t setPTPCmd(int ptpMode, std::vector<float> &target_points, bool isQueued = 1);

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
    uint64_t setEMotor(int index,int insEnabled,float speed,bool isQueued);

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


    float unpackFloatLE(std::vector<u_int8_t>::iterator it);

private:

    std::string _port;

    SerialPort::BaudRate _baud;
    SerialPort::Parity _parity;
    SerialPort::StopBits _stop_bit;
    SerialPort::CharacterSize _character_size;

    SerialPort* _serial_port;

    std::mutex _communication_mt;

    u_int8_t checksumCalc(std::vector<uint8_t> &ctrl_cmd);
    void sendCommand(std::vector<u_int8_t> &ctrl_cmd);
    bool getResponse(std::vector<u_int8_t> &returned_data);
    void packFromFloat(std::vector<float> &value_to_pack, std::vector<u_int8_t> &packed_floats);
    void floatToByte(float float_variable, u_int8_t temp_bytes[]);
    uint64_t getQueuedCmdIndex(std::vector<u_int8_t> data);






};

#endif /* DOBOT_COMMUNICATION_H_ */
