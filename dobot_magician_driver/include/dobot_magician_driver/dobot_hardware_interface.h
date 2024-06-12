#ifndef DOBOT_HARDWARE_INTERFACE_H
#define DOBOT_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <dobot_magician_driver/dobot_driver.h>

class DobotHardwareInterface : public hardware_interface::RobotHW
{
public:
    DobotHardwareInterface(ros::NodeHandle& nh, std::shared_ptr<DobotDriver> dobot_driver);
    ~DobotHardwareInterface();
    void init();
    void read();
    void write();
    ros::Duration getPeriod();
    
private:
    bool commandChanged();  // Method to check if the command has changed

    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::JointStateInterface jnt_state_interface_;

    std::shared_ptr<DobotDriver> dobot_driver_;
    ros::NodeHandle nh_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_position_commands_;
    std::vector<double> last_joint_position_commands_;  // Store the last command

    ros::Time last_time_;
};

#endif
