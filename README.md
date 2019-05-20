# dobot_magician_driver
The dobot_magician_driver provides a ROS interface to communicate with the Dobot Magician.
Please refer to wiki for more detailed instructions on how to install dobot_magician_driver to raspberry pi and interface with Matlab. 
## Dependencies
This package requires lib serial 

```
sudo apt install libserial-dev
```

## Installation
To build from source
```
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/src
git clone https://github.com/gapaul/dobot_magician_driver.git
cd ..
catkin_make
```

Configuring udev rules

```
sudo cp -i  ~/catkin_ws/src/dobot_magician_driver/supporting/43-dobot_magician.rules /etc/udev/rules.d
```
## Usage
Run the following command
```
roslaunch dobot_magician_driver dobot_magician.launch
```
