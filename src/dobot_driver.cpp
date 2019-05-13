#include "dobot_magician_driver/dobot_driver.h"


DobotDriver::DobotDriver(std::string port)
{
    dobot_serial = new DobotCommunication(port);
    dobot_states = new DobotStates();
}

void DobotDriver::getJointAngles(std::vector<double> &joint_angles)
{
    std::vector<u_int8_t> data; // think of a better name
    std::vector<double> pose_data;
//    std::cout << "1" << std::endl;
    if(dobot_serial->getPose(data))
    {
//        std::cout << "2" << std::endl;
        dobot_states->getPose(data, pose_data);
//        std::cout << "3" << std::endl;

        joint_angles = std::vector<double>(pose_data.begin()+4, pose_data.end());
    }

    std::cout << "size:" << joint_angles.size()<<std::endl;


}
