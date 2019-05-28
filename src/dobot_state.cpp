#include <dobot_magician_driver/dobot_state.h>

DobotStates::DobotStates()
{

}

bool DobotStates::unpackPose(std::vector<uint8_t> &data, std::vector<double> &pose)
{

//    unpackPose(data);
    if(data.size() != 32){
        return false;
    }

    pose.clear();
    for(int i = 0; i < 8; ++i){
        pose.push_back(unpackFloat(data.begin() + i*4));
    }
    return true;

}

float DobotStates::unpackFloat(std::vector<uint8_t>::iterator it)
{
    float temp;
    uint8_t b[] = {*it, *(it+1), *(it+2), *(it+3)};
    std::memcpy(&temp, &b, sizeof(temp)); //convert to float from bytes[4]
//    printf("%f\n", temp);
    return temp;
}
