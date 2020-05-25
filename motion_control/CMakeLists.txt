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

add_library(motion_control_hw_lib
  src/hw/basic_robot_hw.cpp
  src/hw/front_steering_hw.cpp
  src/hw/rear_drive_hw.cpp
)

add_library(motion_control_sim_lib
  src/sim/basic_robot_sim.cpp
  src/sim/front_steering_sim.cpp
  src/sim/rear_drive_sim.cpp
)

add_library(motion_control_lib
  src/motion_control.cpp)

target_link_libraries(motion_control_lib motion_control_hw_lib motion_control_sim_lib ${catkin_LIBRARIES})

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
