# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Fixed
- Implemented a patch fix for the dobot homing issue where the serial port was closed at some point during the process (root cause still remains unknown at the moment)
- Minor bugfixes for linear rails, emotors and io state/controls

### Changed
- Restructured entire codebase from the original first release by separating the original driver into a state manager, a controller, and a driver that uses the two components
- Streamlined the driver by adding ros standard messages for controlling and monitoring the robot instead of custom services
- Added custom data types with thread-safe mutexes, for joint states, end effector state, dobot safety status
- Added some supporting MATLAB scripts as a reference for end user
- Added error handlings in critical serial read write functions to prevent unnecessary errors
- Added libusb and a mechanism to scan and check for a specific usb device (in this case the dobot usb port) before starting the driver
- Added high level info logging on ROS side
- Added CHANGELOG

### Removed
- Removed udev rule, as it may affect the ability to support multiple dobot instances in the future

## [0.0.1] - 2021-03-30

- First release (legacy version)
