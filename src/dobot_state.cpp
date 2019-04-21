//Store Lastest States

#include <dobot_magician_driver/dobot_states.h>

DobotStates::DobotStates()
{

}

void DobotStates::unpackPose(std::vector<u_int8_t> &data)
{
    _robot_states.x = unpackFloat(data.begin());
//    float x =       unpackFloat(data.begin());
//    float y =       unpackFloat(data.begin()+4);
//    float z =       unpackFloat(data.begin()+8);
//    float r =       unpackFloat(data.begin()+12);
//    float base =    unpackFloat(data.begin()+16);
//    float reararm = unpackFloat(data.begin()+20);
//    float forearm = unpackFloat(data.begin()+24);
//    float end =     unpackFloat(data.begin()+28);
}


float DobotStates::unpackFloat(std::vector<u_int8_t>::iterator it)
{
    float temp;
    u_int8_t b[] = {*it, *(it+1), *(it+2), *(it+3)};
    std::memcpy(&temp, &b, sizeof(temp));//convert to float from bytes[4]
//    printf("%f\n", temp);
    return temp;
}
