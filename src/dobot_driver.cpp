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

void DobotDriver::run()
{
    update_thread_ = new std::thread(&DobotDriver::driverUpdateThread, this);
}

void DobotDriver::setStateManager(std::shared_ptr<DobotStates> dobot_state_ptr)
{
    dobot_state_ = dobot_state_ptr;
}

void DobotDriver::setController(std::shared_ptr<DobotController> dobot_controller_ptr)
{
    dobot_controller_ = dobot_controller_ptr;
}

void DobotDriver::driverUpdateThread()
{
    while(true)
    {
        
    }
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