#ifndef DOBOT_DRIVER_H_
#define DOBOT_DRIVER_H_

#include "dobot_communication.h"
#include "dobot_state.h"


class DobotDriver{



public:
    DobotCommunication *dobot_serial;
    DobotStates *dobot_states;

    DobotDriver(std::string port);

    bool getJointAngles(std::vector<double> &joint_angles);
    bool getCartesianPos(std::vector<double> &cart_pos);
    bool setJointAngles(std::vector<float> &joint_angles);
    bool setCartesianPos(std::vector<float> &cart_pos);
//    bool setHomeCalibrate()

};

#endif /* DOBOT_DRIVER_H_ */
