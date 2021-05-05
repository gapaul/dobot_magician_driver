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
    if(!dobot_serial_->isConnected())
    {
        return;
    }

    tool_state_ = state;

    dobot_serial_->setEndEffectorSuctionCup(state,state);
}

bool DobotController::getToolState()
{
    if(!dobot_serial_->isConnected())
    {
        return false;
    }

    std::vector<uint8_t> return_data;

    bool result = dobot_serial_->getEndEffectorSuctionCup(return_data);

    if(result && return_data.size() > 3)
    {
        tool_state_ = (bool)return_data.at(3);
    }
    
    return tool_state_;
}

// IO control
void DobotController::setIOState(int address, int multiplex, std::vector<double> data)
{
    if(!dobot_serial_->isConnected())
    {
        return;
    }

    dobot_serial_->setIOMultiplexing(address, multiplex);

    switch (multiplex)
    {
        case IODummy:
            break;

        case IODO:
            dobot_serial_->setIODO(address,(bool)data.at(0));
            break;

        case IOPWM:
            dobot_serial_->setIOPWM(address,(float)data.at(0),(float)data.at(1));
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
    if(!dobot_serial_->isConnected())
    {
        return;
    }

    dobot_serial_->setEMotor(index, is_enabled, speed);
}

// Linear rail positions
void DobotController::setTargetRailWithEEPoses(std::vector<double> target_pose)
{
    if(!dobot_serial_->isConnected())
    {
        return;
    }
    
    target_rail_position_buffer_.add(target_pose); 
    target_rail_position_buffer_.received = true;   
}

bool DobotController::moveToTargetRailPosition()
{
    if(!dobot_serial_->isConnected())
    {
        return false;
    }

    std::vector<double> target;

    target_rail_position_buffer_.mtx.lock();
    target = target_rail_position_buffer_.data.back();
    target_rail_position_buffer_.mtx.unlock();

    std::vector<float> fl_target = std::vector<float>(target.begin(),target.end());
    
    bool result =  dobot_serial_->setPTPWithRailCmd(2, fl_target);

    return result;
}

// Velocity Control 

void DobotController::setTargetJointVelocity(JointConfiguration joint_vel_config)
{
    target_joint_velocity_buffer_.add(joint_vel_config);
    direction_.clear();

    std::vector<float> input_params;
    for(int i = 0; i < joint_vel_config.velocity.size()*2; i++)
    {
        if(i < joint_vel_config.velocity.size())
        {
            input_params.push_back(joint_vel_config.velocity.at(i));
            if(joint_vel_config.velocity.at(i) < 0)
            {
                direction_.push_back(-1);   
            }
            else
            {
                direction_.push_back(1);
            }
        }
        else
        {
            input_params.push_back(joint_vel_config.acceleration.at(i - joint_vel_config.velocity.size()));
        }
        
    }
    dobot_serial_->setJOGJointParams(input_params);
}

bool DobotController::moveWithTargetVelocity()
{
    if(!dobot_serial_->isConnected())
    {
        return false;
    }

    int check = 0;
    bool result = false;
    for(int i = 0; i < direction_.size();i++)
    {
        if(direction_.at(i) > 0)
        {
            result = dobot_serial_->setJOGCmd(1,i*2 + 1,false);
        }
        else
        {
            result = dobot_serial_->setJOGCmd(1,i*2 + 2,false);
        }

        if(result)
        {
            check++;
        }
    }

    if(check == direction_.size())
    {
        return true;
    }
    return false;
}