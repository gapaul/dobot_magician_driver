#ifndef DOBOT_COMMUNICATION_H_
#define DOBOT_COMMUNICATION_H_

#include <iostream>
#include <string>
#include <vector>
#include <cstring>

#include <SerialPort.h>
#include <SerialStream.h>
#include <SerialStreamBuf.h>


class DobotCommunication{

public:
    DobotCommunication(std::string port);

    /// Device information
//    setDeviceSN
//    getDeviceSN
//    setDeviceName
//    setDeviceName
//    getDeviceVersion

    /// Pose
    bool getPose(std::vector<u_int8_t> &returned_data);
//    bool resetPose();

    /// Alarm
//    bool getAlarmState();
//    bool clearAlarmState();

    /// Home
//    setHOMEParams
//    getHOMEParams
//    setHOMECmd

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
    bool setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked);
    bool setEndEffectorSuctionCup(bool is_ctrl_enabled, bool is_sucked, std::vector<u_int8_t> &returned_data);
//    getEndEffectorSuctionCup
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
//    setPTPCmd

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
//    setEMotor

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


    float unpackFloatLE(std::vector<u_int8_t>::iterator it);

private:
    std::string _port;
    SerialPort::BaudRate _baud;
    SerialPort::Parity _parity;
    SerialPort::StopBits _stop_bit;
    SerialPort::CharacterSize _character_size;

    SerialPort* _serial_port;

    void sendCommand(std::vector<u_int8_t> &ctrl_cmd);

    u_int8_t checksumCalc(std::vector<uint8_t> &ctrl_cmd);
    bool getResponse(std::vector<u_int8_t> &returnedData);







};

#endif /* DOBOT_COMMUNICATION_H_ */
