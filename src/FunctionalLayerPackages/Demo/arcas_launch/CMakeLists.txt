cmake_minimum_required(VERSION 2.8.3)
project(arcas_launch)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
#  CATKIN_DEPENDS catec_actions_msgs catec_msgs roscpp
#  DEPENDS 
)
