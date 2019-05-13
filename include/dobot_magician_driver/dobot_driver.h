#ifndef DOBOT_DRIVER_H_
#define DOBOT_DRIVER_H_

#include "dobot_communication.h"
#include "dobot_states.h"


class DobotDriver{



public:
    DobotCommunication *dobot_serial;
    DobotStates *dobot_states;

    DobotDriver(std::string port);

    void getJointAngles(std::vector<double> &joint_angles);
    void getCartesianPos(std::vector<double> &cart_pos);


};

#endif /* DOBOT_DRIVER_H_ */
