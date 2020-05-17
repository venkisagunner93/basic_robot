cmake_minimum_required(VERSION 2.8.3)
project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  tf2_ros
  basic_robot
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

add_library(sim_lib
  src/map.cpp)

add_executable(sim src/simulation.cpp)

target_link_libraries(sim sim_lib ${catkin_LIBRARIES})

#############
## Install ##
#############

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE)

# Install libraries
install(TARGETS sim_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)