#ifndef DOBOT_STATES_H_
#define DOBOT_STATES_H_

#include <iostream>
#include <string>
#include <vector>
#include <cstring>

#include <SerialPort.h>
#include <SerialStream.h>
#include <SerialStreamBuf.h>


class DobotStates{

public:
    struct Pose
    {
        float x;
        float y;
        float z;
        float r;
        std::vector<float> joint_angle;

    };

    DobotStates();
//    void unpackPose(std::vector<u_int8_t> &data);
    void unpackPose(std::vector<u_int8_t> &data, std::vector<double> &pose);


private:


    Pose _pose;

    struct HOMEParams
    {

    }_home_params;


    struct HOMECmd
    {

    }_home_cmd;

    struct WIFIGateway
    {

    }_wifi_gateway;

    struct HHTTrigMode
    {

    }_hht_trig_mode;

    struct EndEffectorParams
    {

    }_end_effector_params;

    struct JOGJointParams
    {

    }_jog_joint_params;

    struct JOGCoordinateParams
    {

    }_jog_coordinate_params;

    struct JOGCommonParams
    {

    }_jog_common_params;

    struct JOGCmd
    {

    }_jog_cmd;

    struct JOGLParams
    {

    }_jog_l_params;

    float unpackFloat(std::vector<u_int8_t>::iterator it);


};

#endif /* DOBOT_STATES_H_ */


