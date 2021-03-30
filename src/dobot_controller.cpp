#include "dobot_magician_driver/dobot_controller.h"

DobotController::DobotController()
{

}

DobotController::~DobotController()
{
    
}

void DobotController::init(std::shared_ptr<DobotCommunication> dobot_serial_ptr)
{
    dobot_serial_ = dobot_serial_ptr;
}

void DobotController::run()
{

}

void DobotController::setTargetJointConfiguration(JointConfiguration target_joint_config)
{
    target_joint_config_buffer_.mtx.lock();

    target_joint_config_buffer_.joint_data.push_back(target_joint_config);
    
    if(target_joint_config_buffer_.joint_data.size() > MAX_BUFFER_SIZE)
    {
        target_joint_config_buffer_.joint_data.pop_front();
    }
    target_joint_config_buffer_.received = true;

    target_joint_config_buffer_.mtx.unlock();
}

void DobotController::setTargetEndEffectorPose(Pose target_end_effector_pose)
{
    target_ee_pose_buffer_.mtx.lock();

    target_ee_pose_buffer_.pose_data.push_back(target_end_effector_pose);

    if(target_ee_pose_buffer_.pose_data.size() > MAX_BUFFER_SIZE)
    {
        target_ee_pose_buffer_.pose_data.pop_front();
    }

    target_ee_pose_buffer_.received = true;
    
    target_ee_pose_buffer_.mtx.unlock();
}

bool DobotController::moveToTargetJoint()
{
    if(!dobot_serial_->isConnected())
    {
        return false;
    }
    std::vector<float> target_joint_config;
    
    target_joint_config_buffer_.mtx.lock();
    
    target_joint_config = std::vector<float>(target_joint_config_buffer_.joint_data.back().position.begin(),
                                             target_joint_config_buffer_.joint_data.back().position.end());
    
    target_joint_config_buffer_.mtx.unlock();

    bool result = dobot_serial_->setPTPCmd(4,target_joint_config,1);

    return result;
}

bool DobotController::moveToTargetPose()
{
    if(!dobot_serial_->isConnected())
    {
        return false;
    }

    std::vector<float> target_pose;
    
    target_ee_pose_buffer_.mtx.lock();
    
    target_pose.push_back(target_ee_pose_buffer_.pose_data.back().x);
    target_pose.push_back(target_ee_pose_buffer_.pose_data.back().y);
    target_pose.push_back(target_ee_pose_buffer_.pose_data.back().z);
    target_pose.push_back(target_ee_pose_buffer_.pose_data.back().theta);

    target_ee_pose_buffer_.mtx.unlock();

    bool result = dobot_serial_->setPTPCmd(2,target_pose);

    return result;
}

// Tool control
void DobotController::setToolState(bool state)
{
    tool_state_ = state;

    dobot_serial_->setEndEffectorSuctionCup(state,state);
}

bool DobotController::getToolState()
{
    std::vector<uint8_t> return_data;
    dobot_serial_->getEndEffectorSuctionCup(return_data);
    tool_state_ = (bool)return_data.at(3);

    return tool_state_;
}

// IO control
void DobotController::setIOState(int address, int multiplex, std::vector<double> data)
{
    dobot_serial_->setIOMultiplexing(address, multiplex);

    switch (multiplex)
    {
        case IODummy:
            break;

        case IODO:
            dobot_serial_->setIODO(address,(bool)data.at(0));
            break;

        case IOPWM:
            dobot_serial_->setIOPWM(address,data.at(0),data.at(1));
            break;

        case IODI:
            break;

        case IOADC:
            break;

        case IODIPU:
            break;

        case IODIPD:
            break;

        default:
            break;
    }
}

// EMotor Control
void DobotController::setEMotorSpeed(int index, bool is_enabled, int speed)
{
    dobot_serial_->setEMotor(index, is_enabled, speed);
}
