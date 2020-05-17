cmake_minimum_required(VERSION 3.0.2)
project(rear_drive_state_interface)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(include 
  ${basic_robot_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
