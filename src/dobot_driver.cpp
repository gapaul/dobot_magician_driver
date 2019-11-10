#include "dobot_magician_driver/dobot_driver.h"
#include <iostream>
#include <thread>

// DobotDriver
DobotDriver::DobotDriver(std::string port)
{
    _dobot_serial = new DobotCommunication(port);
    _dobot_states = new DobotStates();

    _is_on_linear_rail = false;

}

bool DobotDriver::getCurrentConfiguration(std::vector<double> &cart_pos, std::vector<double> &joint_angles)
{
    std::vector<uint8_t> data;
    std::vector<double> pose_data;

    if(_dobot_serial->getPose(data))
    {
        if(_dobot_states->unpackPose(data, pose_data)){
            cart_pos = std::vector<double>(pose_data.begin(), pose_data.begin()+4);
            joint_angles = std::vector<double>(pose_data.begin()+4, pose_data.end());
            return true;
        }
    }

    return false;
}

bool DobotDriver::getJointAngles(std::vector<double> &joint_angles)
{
    std::vector<uint8_t> data;
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
    std::vector<uint8_t> data;
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
    if(_dobot_serial->setPTPCmd(4,joint_angles))
    {
        return true;

    }
    return false;
}

bool DobotDriver::setCartesianPos(std::vector<float> &cart_pos)
{
    if(_dobot_serial->setPTPCmd(2,cart_pos))
    {
        return true;
    }
    return false;
}

bool DobotDriver::setGripper(bool is_ctrl_enabled, bool is_gripped)
{
    if(_dobot_serial->setEndEffectorGripper(is_ctrl_enabled, is_gripped))
    {
        return true;
    }
    return false;
}

bool DobotDriver::setSuctionCup(bool is_ctrl_enabled, bool is_sucked)
{
    if(_dobot_serial->setEndEffectorSuctionCup(is_ctrl_enabled, is_sucked))
    {
        return true;
    }
    return false;
}

/*
 *  CP COMMANDS
 */

bool DobotDriver::setCPParams(std::vector<float> &cp_params, bool real_time_track)
{
    if (_dobot_serial->setCPParams(cp_params, real_time_track,0))
	{
		return true;
	}
	return false;
}

bool DobotDriver::getCPParams(std::vector<float> &cp_params, uint8_t &real_time_track)
{
    std::vector<uint8_t> data;

    if(_dobot_serial->getCPParams(data))
    {
        if(_dobot_states->unpackCPParams(data, cp_params, real_time_track))
        {
            return true;
        }
    }
    return false;
}

bool DobotDriver::setCPCmd(std::vector<float> &cp_cmd, bool cp_mode)
{
	uint64_t queue_command_index;
    if (_dobot_serial->setCPCmd(cp_cmd, cp_mode,queue_command_index,1))
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
    if(_dobot_serial->setIOMultiplexing(address,multiplex))
    {
        return true;
    }
    return false;
}

bool DobotDriver::getIOMultiplexing(int address, int &multiplex)
{
    std::vector<uint8_t> data;

    if(_dobot_serial->getIOMultiplexing(address,data))
    {
        multiplex = (int) data.at(3);
        return true;
    }

    return false;
}

bool DobotDriver::setIODigitalOutput(int address, bool level)
{
    if(_dobot_serial->setIODO(address,level))
    {
        return true;
    }
    return false;
}

bool DobotDriver::getIODigitalOutput(int address, bool &level)
{
    std::vector<uint8_t> data;

    if(_dobot_serial->getIODO(address,data))
    {
        level = (bool) data.at(3);
        return true;
    }

    return false;
}

bool DobotDriver::setIOPWM(int address, float frequency, float duty_cycle)
{
    if(_dobot_serial->setIOPWM(address,frequency,duty_cycle))
    {
        return true;
    }
    return false;
}

bool DobotDriver::getIODigitalInput(int address, bool &level)
{
    std::vector<uint8_t> data;

    if(_dobot_serial->getIODI(address,data))
    {
        level = (bool) data.at(3);
        return true;
    }

    return false;
}

bool DobotDriver::getIOAnalogInput(int address, int &value)
{
    std::vector<uint8_t> data;

    if(_dobot_serial->getIOADC(address,data))
    {
        value = (data.at(4)<<8) | data.at(3);
        return true;
    }

    return false;
}

bool DobotDriver::setEMotor(int index,bool is_enabled,int32_t speed)
{
    if(_dobot_serial->setEMotor(index,is_enabled,speed))

    {
        return true;
    }

    return false;
}

/* QUEUED EXECUTION CONTROL COMMANDS */

bool DobotDriver::setQueuedCmdStartExec(void)
{
    if (_dobot_serial->setQueuedCmdStartExec())
    {
        return true;
    }
    return false;
}

bool DobotDriver::setQueuedCmdStopExec(void)
{
    if (_dobot_serial->setQueuedCmdStopExec())
    {
        return true;
    }
    return false;
}

bool DobotDriver::setQueuedCmdForceStopExec(void)
{
    if (_dobot_serial->setQueuedCmdForceStopExec())
    {
        return true;
    }
    return false;
}

bool DobotDriver::setQueuedCmdClear(void)
{
    if (_dobot_serial->setQueuedCmdClear())
    {
        return true;
    }
    return false;
}

/* E-STOP */

bool DobotDriver::stopAllIO(void)
{
    int check_IO = 0;

    for (int i = 1; i<=20;i++)
    {
        if (_dobot_serial->setIOMultiplexing(i,0,0))
        {
            check_IO++;
        }
    }

    if (check_IO != 20)
    {
        return false;
    }

    return true;
}

bool DobotDriver::setEStop(void)
{
    bool stop_pump;

    bool stop_queued = setQueuedCmdForceStopExec();
    setQueuedCmdClear();

    if (!_dobot_serial->setEndEffectorSuctionCup(0,0,0) || !_dobot_serial->setEndEffectorGripper(0,0,0))
    {
        stop_pump = false;
        return false;
    }
    else stop_pump = true;

    bool stop_IO = stopAllIO();

    _dobot_serial->setEMotor(0,false,0);//turn off stepper 1
    _dobot_serial->setEMotor(1,false,0);//turn off stepper 2

    if (stop_queued && stop_pump && stop_IO)
    {
        _is_e_stopped = true;
        return true;
    }
    return false;
}

bool DobotDriver::isEStopped(void)
{
    return _is_e_stopped;
}

/* LINEAR RAIL */

bool DobotDriver::setLinearRailStatus(bool is_enabled)
{
    if (_dobot_serial->setLinearRailStatus(is_enabled,0,0))
    {
        if(is_enabled)
        {
            _is_on_linear_rail = true;
        }
        else _is_on_linear_rail = false;

        return true;
    }
    return false;
}

bool DobotDriver::setCartesianPosWithRail(std::vector<float> &cart_pos)
{
    if (!isOnLinearRail())
    {
        return false;
    }

    if(_dobot_serial->setPTPWithRailCmd(2,cart_pos))
    {
        return true;
    }
    return false;
}

bool DobotDriver::isOnLinearRail(void)
{
    return _is_on_linear_rail;
}

void DobotDriver::initialiseDobot()
{
    _is_e_stopped = false;
    _dobot_serial->setQueuedCmdClear();
    _dobot_serial->setQueuedCmdStartExec();

    _dobot_serial->setEMotor(0,false,0);//turn off stepper 1
    _dobot_serial->setEMotor(1,false,0);//turn off stepper 2

    stopAllIO();
    std::vector<float> start_joint_angles={0,0.4,0.3,0};
    setJointAngles(start_joint_angles);
    setLinearRailStatus(isOnLinearRail());
    std::this_thread::sleep_for(std::chrono::seconds(2));
    _dobot_serial->setHOMECmd(); //create setter for this to access from ros wrapper
}
