#include <dobot_magician_driver/dobot_states.h>

DobotStates::DobotStates()
{

}

void DobotStates::getPose(std::vector<u_int8_t> &data, std::vector<double> &pose)
{

//    unpackPose(data);
    pose.clear();
    for(int i = 0; i < 8; ++i){
        pose.push_back(unpackFloat(data.begin()+ i*4));
    }

}

void DobotStates::unpackPose(std::vector<u_int8_t> &data)
{

    _pose.joint_angle.clear();
    _pose.x = unpackFloat(data.begin());
    _pose.y = unpackFloat(data.begin()+4);
    _pose.z = unpackFloat(data.begin()+8);
    _pose.r = unpackFloat(data.begin()+12);
    _pose.joint_angle.push_back(unpackFloat(data.begin()+16));
    _pose.joint_angle.push_back(unpackFloat(data.begin()+20));
    _pose.joint_angle.push_back(unpackFloat(data.begin()+24));
    _pose.joint_angle.push_back(unpackFloat(data.begin()+28));

}


float DobotStates::unpackFloat(std::vector<u_int8_t>::iterator it)
{
    float temp;
    u_int8_t b[] = {*it, *(it+1), *(it+2), *(it+3)};
    std::memcpy(&temp, &b, sizeof(temp)); //convert to float from bytes[4]
//    printf("%f\n", temp);
    return temp;
}
