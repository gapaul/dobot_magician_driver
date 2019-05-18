#include "dobot_magician_driver/dobot_driver.h"

// DobotDriver
DobotDriver::DobotDriver(std::string port)
{
    _dobot_serial = new DobotCommunication(port);
    _dobot_states = new DobotStates();

}

bool DobotDriver::getJointAngles(std::vector<double> &joint_angles)
{
    std::vector<u_int8_t> data;
    std::vector<double> pose_data;

    if(_dobot_serial->getPose(data))
    {
        if(_dobot_states->unpackPose(data, pose_data)){
            joint_angles = std::vector<double>(pose_data.begin()+4, pose_data.end());
            return true;
        }
    }

    return false;
}

bool DobotDriver::getCartesianPos(std::vector<double> &cart_pos)
{
    std::vector<u_int8_t> data;
    std::vector<double> pose_data;

    if(_dobot_serial->getPose(data))
    {
        if(_dobot_states->unpackPose(data, pose_data)){
            cart_pos = std::vector<double>(pose_data.begin(), pose_data.begin()+4);
            return true;
        }
    }

    return false;
}

bool DobotDriver::setJointAngles(std::vector<float> &joint_angles)
{
    if(_dobot_serial->setPTPCmd(4,joint_angles) >= -1){
        return true;

    }
    return false;
}

bool DobotDriver::setCartesianPos(std::vector<float> &cart_pos)
{
    if(_dobot_serial->setPTPCmd(2,cart_pos) >= -1){

        return true;

    }

    return false;
}

bool DobotDriver::setGripper(bool is_ctrl_enabled, bool is_gripped)
{
    if(_dobot_serial->setEndEffectorGripper(is_ctrl_enabled, is_gripped) >= -1){

        return true;

    }

    return false;
}

bool DobotDriver::setSuctionCup(bool is_ctrl_enabled, bool is_sucked)
{
    if(_dobot_serial->setEndEffectorSuctionCup(is_ctrl_enabled,is_sucked) >= -1){

        return true;

    }

    return false;
}

bool DobotDriver::setCPParams(std::vector<float> &CPParams, int realtimeTrack, bool is_queued)
{
	if (_dobot_serial->setCPParams(CPParams, realtimeTrack, is_queued) >= -1)
	{
		return true;
	}
	return false;
}

// bool DobotDriver::getCPParams(std::vector<u_int8_t> &CPParams_data, bool is_queued)
// {
//     if(_dobot_serial->setCPParams(CPParams_data,is_queued) >=-1)
//     {
//         return true;
//     }
//     return false;
// }

bool DobotDriver::setCPCmd(std::vector<float> &CPCmd, int cpMode, bool is_queued)
{
	if (_dobot_serial->setCPCmd(CPCmd, cpMode, is_queued) >= -1)
	{
		return true;
	}
	return false;
}

/*
 *  I/O COMMANDS
 */

bool DobotDriver::setIOMultiplexing(int address, int multiplex)
{
    if(_dobot_serial->setIOMultiplexing(address,multiplex) >= -1){
      return true;
    }
    return false;
}

bool DobotDriver::getIOMultiplexing(int address, int &multiplex)
{
    std::vector<u_int8_t> data;

    if(_dobot_serial->getIOMultiplexing(address,data))
    {
        multiplex = (int) data.at(1);
        return true;
    }

    return false;
}

bool DobotDriver::setIODigitalOutput(int address, bool level)
{
    if(_dobot_serial->setIODO(address,level) >= -1){
      return true;
    }
    return false;
}

bool DobotDriver::getIODigitalOutput(int address, bool &level)
{
    std::vector<u_int8_t> data;

    if(_dobot_serial->getIODO(address,data))
    {
        level = (bool) data.at(1);
        return true;
    }

    return false;
}

bool DobotDriver::setIOPWM(int address, float frequency, float duty_cycle)
{
    if(_dobot_serial->setIOPWM(address,frequency,duty_cycle) >= -1){
      return true;
    }
    return false;
}

bool DobotDriver::getIODigitalInput(int address, bool &level)
{
    std::vector<u_int8_t> data;

    if(_dobot_serial->getIODI(address,data))
    {
        level = (bool) data.at(1);
        return true;
    }

    return false;
}

void DobotDriver::initialiseDobot()
{
    _dobot_serial->setHOMECmd(1); //create setter for this to access from ros wrapper
    //    dobot_serial->setEMotor(0,0,5000,true);//turn off stepper 1
    //    dobot_serial->setEMotor(1,0,5000,true);//turn off stepper 2
}
