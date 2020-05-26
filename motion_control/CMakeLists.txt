cmake_minimum_required(VERSION 3.0.2)
project(motion_control)

find_package(catkin REQUIRED COMPONENTS
  basic_robot
  controller_manager
  rear_drive_state_interface
  rear_drive_command_interface
  front_steering_state_interface
  front_steering_command_interface
  motor_driver
  tf2_ros
  roscpp
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS basic_robot motor_driver
)

###########
## Build ##
###########
include_directories(include 
  ${basic_robot_INCLUDE_DIRS}
  ${motor_driver_INCLUDE_DIRS}
  ${rear_drive_state_interface_INCLUDE_DIRS}
  ${rear_drive_command_interface_INCLUDE_DIRS}
  ${front_steering_state_interface_INCLUDE_DIRS}
  ${front_steering_command_interface_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS})

add_library(motion_control_lib
  src/motion_control.cpp
  src/basic_robot_base.cpp
  src/basic_robot_sim.cpp
  src/basic_robot_hw.cpp)

add_executable(motion_control_hw src/motion_control_hw_main.cpp)
target_link_libraries(motion_control_hw motion_control_lib ${catkin_LIBRARIES})

add_executable(motion_control_sim src/motion_control_sim_main.cpp)
target_link_libraries(motion_control_sim motion_control_lib ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install libraries
install(TARGETS motion_control_lib
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS motion_control_hw
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
