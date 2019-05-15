#include "dobot_magician_driver/dobot_driver.h"


DobotDriver::DobotDriver(std::string port)
{
    dobot_serial = new DobotCommunication(port);
    dobot_states = new DobotStates();

    dobot_serial->setHOMECmd(1); //create setter for this to access from ros wrapper

}

bool DobotDriver::getJointAngles(std::vector<double> &joint_angles)
{
    std::vector<u_int8_t> data; // think of a better name
    std::vector<double> pose_data;

    if(dobot_serial->getPose(data))
    {
        dobot_states->unpackPose(data, pose_data);
        joint_angles = std::vector<double>(pose_data.begin()+4, pose_data.end());
        return true;
    }

    return false;
}

bool DobotDriver::getCartesianPos(std::vector<double> &cart_pos)
{
    std::vector<u_int8_t> data;
    std::vector<double> pose_data;

    if(dobot_serial->getPose(data))
    {
        dobot_states->unpackPose(data, pose_data);
        cart_pos = std::vector<double>(pose_data.begin(), pose_data.begin()+4);
        return true;
    }

    return false;
}

bool DobotDriver::setJointAngles(std::vector<float> &joint_angles)
{
    if(dobot_serial->setPTPCmd(4,joint_angles)){
        return true;

    }
    return false;
}

bool DobotDriver::setCartesianPos(std::vector<float> &cart_pos)
{
    if(dobot_serial->setPTPCmd(2,cart_pos)){

        return true;

    }
    return false;
}

