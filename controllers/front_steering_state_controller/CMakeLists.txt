cmake_minimum_required(VERSION 3.0.2)
project(front_steering_state_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface  
  basic_robot
  ackermann_msgs
  front_steering_state_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

add_library(front_steering_state_controller
  src/front_steering_state_controller.cpp)

###########
## Build ##
###########
include_directories(include 
  ${basic_robot_INCLUDE_DIRS}
  ${rear_drive_interface_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

# Install libraries
install(TARGETS front_steering_state_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES front_steering_state_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)