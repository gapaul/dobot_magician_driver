# DOBOT Magician Driver
The dobot_magician_driver provides a ROS interface to communicate with the Dobot Magician.
Please refer to wiki for more detailed instructions on how to install dobot_magician_driver to raspberry pi and interface with Matlab. 

## Features
Current supported features are listed below:
- Joint position-based control
- Joint state publisher
- Cartesian position-based control
- End effector cartesian state publisher
- Tool state control: Currently supporting gripper and suction cup effectors
- IO states control
- Dobot safety status publisher
- Software EStop
- Conveyor belt and linear rail control

## Contents
This repository contains the dobot_magician_driver and a couple of example scripts, such as:
- MATLAB example: A quick and easy way to communicate with the driver to control the robot, providing that the driver is already running. This example should only be used as a reference for using the driver with MATLAB. 

## Dependencies
This package requires libserial and lubusb-1.0. Please refer to the wiki for installation instruction.

## Installation
Please refer to the wiki for installation instruction.

## Usage
Please refer to the wiki for usage instruction.

## To do
- Add example script for Python and C++