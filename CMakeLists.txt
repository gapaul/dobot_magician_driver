cmake_minimum_required(VERSION 3.5)
project(dobot_magician_driver)

## Compile as C++14, supported in ROS2 Jazzy and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)
pkg_check_modules(LIBUSB1 REQUIRED libusb-1.0)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
  ${LIBUSB1_INCLUDE_DIRS}
)

set(SOURCES
  src/dobot_ros_wrapper.cpp
  src/dobot_communication.cpp
  src/dobot_state.cpp
  src/dobot_controller.cpp
  src/dobot_driver.cpp
)

add_executable(dobot_magician_driver ${SOURCES})

ament_target_dependencies(dobot_magician_driver
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  trajectory_msgs
)

target_link_libraries(dobot_magician_driver
  ${SERIAL_LIBRARIES}
  ${LIBUSB1_LIBRARIES}
)

install(TARGETS
  dobot_magician_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# If you have launch files, add this section
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
