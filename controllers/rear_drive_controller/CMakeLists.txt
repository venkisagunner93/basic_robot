cmake_minimum_required(VERSION 3.0.2)
project(rear_drive_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  basic_robot
  controller_interface
  rear_drive_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

add_library(rear_drive_controller
  src/rear_drive_controller.cpp)

###########
## Build ##
###########
include_directories(include 
  ${basic_robot_INCLUDE_DIRS}
  ${rear_drive_interface_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})