cmake_minimum_required(VERSION 3.0.2)
project(rrbot_bringup)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package(
)

#############
## Install ##
#############

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
