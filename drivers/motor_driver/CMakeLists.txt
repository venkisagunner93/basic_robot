cmake_minimum_required(VERSION 3.0.2)
project(motor_driver)

find_package(catkin REQUIRED COMPONENTS
  basic_robot
  roscpp
)

find_package(gazebo REQUIRED)

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
  ${GAZEBO_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

# Install libraries
# install(TARGETS  
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# Install header files
# install(DIRECTORY include/${PROJECT_NAME}/ 
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
