cmake_minimum_required(VERSION 3.0.2)
project(motor_driver)

find_package(catkin REQUIRED COMPONENTS
  basic_robot
  rear_drive_command_interface
  rear_drive_state_interface
  controller_manager
  roscpp
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES motor_driver_sim motor_driver_hw
 CATKIN_DEPENDS basic_robot rear_drive_state_interface rear_drive_command_interface
)

###########
## Build ##
###########
include_directories(include 
  ${basic_robot_INCLUDE_DIRS}
  ${rear_drive_state_interface_INCLUDE_DIRS}
  ${rear_drive_command_interface_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS})

add_library(motor_driver_sim
  src/motor_driver_interface.cpp
  src/motor_driver_sim_interface.cpp)

add_library(motor_driver_hw
  src/motor_driver_interface.cpp
  src/motor_driver_hw_interface.cpp)

add_executable(motor_sim src/motor_sim_main.cpp)
target_link_libraries(motor_sim motor_driver_sim ${catkin_LIBRARIES})

add_executable(motor_hw src/motor_hw_main.cpp)
target_link_libraries(motor_hw motor_driver_hw ${catkin_LIBRARIES})

#############
## Install ##
#############

# Install libraries
install(TARGETS motor_driver_sim motor_driver_hw
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS motor_sim motor_hw
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
