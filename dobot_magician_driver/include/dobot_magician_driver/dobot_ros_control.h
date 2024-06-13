#ifndef DOBOT_CONTROL_NODE_H
#define DOBOT_CONTROL_NODE_H

#include <ros/ros.h>
#include <thread>
#include <controller_manager/controller_manager.h>
#include "dobot_magician_driver/dobot_hardware_interface.h"
#include "dobot_magician_driver/dobot_driver.h"

void initialiseDobotHardware(std::shared_ptr<DobotDriver> dobot_driver, std::string port);

#endif // DOBOT_CONTROL_NODE_H