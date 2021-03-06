#include "dobot_magician_driver/dobot_ros_wrapper_interface.h"

DobotRosWrapperInterface::DobotRosWrapperInterface()
{

}

DobotRosWrapperInterface::~DobotRosWrapperInterface()
{

}

void DobotRosWrapperInterface::setDriver(DobotDriver *driver)
{
    *driver_  = driver;
}

DobotDriver DobotRosWrapperInterface::getDriver()
{
    return *driver_;
}

