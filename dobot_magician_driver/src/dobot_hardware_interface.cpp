#include "dobot_magician_driver/dobot_hardware_interface.h"

DobotHardwareInterface::DobotHardwareInterface(ros::NodeHandle& nh, std::shared_ptr<DobotDriver> dobot_driver)
    : nh_(nh), dobot_driver_(dobot_driver), joint_names_{"magician_joint_1", "magician_joint_2", "magician_joint_3", "magician_joint_4"}
{
    init();
}

DobotHardwareInterface::~DobotHardwareInterface()
{
}

void DobotHardwareInterface::init()
{
    joint_positions_.resize(joint_names_.size());
    joint_velocities_.resize(joint_names_.size());
    joint_efforts_.resize(joint_names_.size());
    joint_position_commands_.resize(joint_names_.size());
    last_joint_position_commands_.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); i++)
    {
        hardware_interface::JointStateHandle state_handle(joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]);
        jnt_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle vel_handle(state_handle, &joint_position_commands_[i]);
        position_joint_interface_.registerHandle(vel_handle);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&position_joint_interface_);

    last_time_ = ros::Time::now();
}

void DobotHardwareInterface::read()
{
    JointConfiguration current_joint_config = dobot_driver_->getCurrentJointConfiguration();
    joint_positions_[0] = current_joint_config.position[0]*M_PI/180;
    joint_velocities_[0] = current_joint_config.velocity[0]*M_PI/180;
    joint_positions_[1] = current_joint_config.position[1]*M_PI/180;
    joint_velocities_[1] = current_joint_config.velocity[1]*M_PI/180;
    joint_positions_[2] = (current_joint_config.position[2]-current_joint_config.position[1])*M_PI/180;
    joint_velocities_[2] = current_joint_config.velocity[2]*M_PI/180;
    joint_positions_[3] = current_joint_config.position[3]*M_PI/180;
    joint_velocities_[3] = current_joint_config.velocity[3]*M_PI/180;
}

void DobotHardwareInterface::write()
{
    if (commandChanged())  // Only write if the command has changed
    {
        std::vector<double> position_commands(joint_position_commands_.size());

        position_commands[0] = joint_position_commands_[0] * 180 / M_PI;
        position_commands[1] = joint_position_commands_[1] * 180 / M_PI;
        position_commands[2] = joint_position_commands_[2] * 180 / M_PI;
        position_commands[3] = joint_position_commands_[3] * 180 / M_PI;

        dobot_driver_->setTargetJointConfiguration(position_commands);
        bool success = dobot_driver_->moveToTargetJointConfiguration();

        if (!success)
        {
            ROS_ERROR("Failed to send position commands to Dobot.");
        }

        last_joint_position_commands_ = joint_position_commands_;  // Update the last commands
    }
}

bool DobotHardwareInterface::commandChanged()
{
    for (size_t i = 0; i < joint_position_commands_.size(); i++)
    {
        if (joint_position_commands_[i] != last_joint_position_commands_[i])
        {
            return true;  // Command has changed
        }
    }
    return false;  // No change in command
}

ros::Duration DobotHardwareInterface::getPeriod()
{
    ros::Time current_time = ros::Time::now();
    ros::Duration period = current_time - last_time_;
    last_time_ = current_time;
    return period;
}
