cmake_minimum_required(VERSION 2.8.3)
project(basic_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  LIBRARIES system
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(system
  src/system/kinematics.cpp)

#############
## Install ##
#############

# Install libraries
install(TARGETS system
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/system/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/system
)