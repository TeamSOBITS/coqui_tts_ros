cmake_minimum_required(VERSION 3.10.2)
project(coqui_tts_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sobits_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coqui_tts_ros
#  CATKIN_DEPENDS rospy sobits_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
