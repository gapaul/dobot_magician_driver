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

    DobotStates();
    void unpackPose(std::vector<u_int8_t> &data);

private:

    struct robot_states
    {
        float x;
        float y;
    }_robot_states;



    float unpackFloat(std::vector<u_int8_t>::iterator it);


};

#endif /* DOBOT_STATES_H_ */


