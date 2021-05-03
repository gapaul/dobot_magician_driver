# dobot_magician_driver
The dobot_magician_driver provides a ROS interface to communicate with the Dobot Magician.
Please refer to wiki for more detailed instructions on how to install dobot_magician_driver to raspberry pi and interface with Matlab. 
## Dependencies
This package requires lib serial. Please refer to the wiki for installation instruction.

## Installation
Please refer to the wiki for installation instruction.

Configuring udev rules

```
sudo cp -i  ~/catkin_ws/src/dobot_magician_driver/supporting/43-dobot_magician.rules /etc/udev/rules.d
```
## Usage
Run the following command
```
roslaunch dobot_magician_driver dobot_magician.launch
```
