#include "dobot_magician_driver/dobot_driver.h"

DobotDriver::DobotDriver()
{

}

DobotDriver::~DobotDriver()
{

}

void DobotDriver::init(std::shared_ptr<DobotStates> dobot_state_ptr, std::shared_ptr<DobotController> dobot_controller_ptr)
{
    dobot_state_ = dobot_state_ptr;
    dobot_controller_ = dobot_controller_ptr;
}

void DobotDriver::setStateManager(std::shared_ptr<DobotStates> dobot_state_ptr)
{
    dobot_state_ = dobot_state_ptr;
}

void DobotDriver::setController(std::shared_ptr<DobotController> dobot_controller_ptr)
{
    dobot_controller_ = dobot_controller_ptr;
}

void DobotDriver::initialiseRobot()
{
    dobot_state_->initialiseRobot();
}

// not a very good way to do this
bool DobotDriver::isRobotAtTarget()
{      
    return at_target_;
}

bool DobotDriver::isRobotInMotion()
{
    return in_motion_;
}

void DobotDriver::isRobotOnLinearRail(bool is_on_rail)
{
    is_on_rail_ = is_on_rail;

    dobot_state_->setOnRail(is_on_rail_);
}

JointConfiguration DobotDriver::getCurrentJointConfiguration()
{
    return dobot_state_->getRobotCurrentJointConfiguration();
}

Pose DobotDriver::getCurrentEndEffectorPose()
{
    return dobot_state_->getRobotCurrentEndEffectorPose();
}

void DobotDriver::setTargetJointConfiguration(JointConfiguration target_joint_config)
{
    dobot_controller_->setTargetJointConfiguration(target_joint_config);
}

void DobotDriver::setTargetEndEffectorPose(Pose target_end_effector_pose)
{
    dobot_controller_->setTargetEndEffectorPose(target_end_effector_pose);
}

void DobotDriver::setTargetJointConfiguration(std::vector<double> target_joint_config_vec)
{
    JointConfiguration joint_config;
    joint_config.position = target_joint_config_vec;

    dobot_controller_->setTargetJointConfiguration(joint_config);
}

void DobotDriver::setTargetEndEffectorPose(std::vector<double> target_end_effector_pose_vec)
{
    Pose end_effector_pose ;
    end_effector_pose.convert(target_end_effector_pose_vec);
    dobot_controller_->setTargetEndEffectorPose(end_effector_pose);
}

bool DobotDriver::moveToTargetJointConfiguration()
{
    bool result = dobot_controller_->moveToTargetJoint();
    return result;
}

bool DobotDriver::moveToTargetEndEffectorPose()
{
    bool result = dobot_controller_->moveToTargetPose();
    return result;
}

void DobotDriver::driverUpdateThread()
{
    JointConfiguration current_config;
    int check_config = 0;

    while(true)
    {
        current_config = dobot_state_->getRobotCurrentJointConfiguration();
    
        for(int i = 0; i < current_config.position.size(); i++)
        {
            if(abs(current_config.position.at(i) - current_target_config_.position.at(i)) >= JOINT_ERROR)
            {
                at_target_ = false;
                in_motion_ = true;
            }
            else
            {
                check_config++;
            }
        }

        if(check_config == 4)
        {
            at_target_ = true;
            in_motion_ = false;

            check_config = false;
        }
        
    }
}
