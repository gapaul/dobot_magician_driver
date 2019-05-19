# dobot_magician_driver
The dobot_magician_driver provides a ROS interface to communicate with the Dobot Magician.

## Dependencies
This package requires lib serial 

```
sudo apt install libserial-dev

```

## Installation
Installing dobot_magician_driver

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

