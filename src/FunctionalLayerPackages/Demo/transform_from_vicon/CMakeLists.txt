cmake_minimum_required(VERSION 2.8.3)
project(transform_from_vicon)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  arcas_msgs
  arcas_actions_msgs
  tf
  geometry_msgs
  sensor_msgs
  functions
  RVO2-3D_unstable
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_actions_msgs arcas_msgs roscpp functions RVO2-3D_unstable
#  DEPENDS
)

###########
## Build ##
###########

include_directories(
   include ${catkin_INCLUDE_DIRS}
)

add_executable(node src/scan_to_octomap_pelican.cpp)
add_dependencies(node arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(node ${catkin_LIBRARIES})
