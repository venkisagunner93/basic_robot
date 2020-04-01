cmake_minimum_required(VERSION 2.8.3)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(planner src/planner.cpp)

target_link_libraries(planner ${catkin_LIBRARIES})

