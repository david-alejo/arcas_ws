cmake_minimum_required(VERSION 2.8.3)
project(RVO2-3D)

find_package(catkin REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES RVO2-3D
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include include/RVO2-3D/)
add_library(${PROJECT_NAME} src/Agent.cpp src/KdTree.cpp src/RVOSimulator.cpp)