cmake_minimum_required(VERSION 2.8.3)
project(simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  tf2_ros
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(sim_lib
  src/kinematics.cpp
  src/map.cpp)

add_executable(sim src/simulation.cpp)

target_link_libraries(sim sim_lib ${catkin_LIBRARIES})