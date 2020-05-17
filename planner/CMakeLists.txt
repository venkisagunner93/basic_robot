cmake_minimum_required(VERSION 2.8.3)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  tf2_ros
  basic_robot
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

add_executable(planner src/planner.cpp)

target_link_libraries(planner ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)