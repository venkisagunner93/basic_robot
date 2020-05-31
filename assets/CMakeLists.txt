cmake_minimum_required(VERSION 2.8.3)
project(assets)

find_package(catkin REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(DIRECTORY config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)