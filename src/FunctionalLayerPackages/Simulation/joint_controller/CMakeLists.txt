cmake_minimum_required(VERSION 2.8.3)
project(joint_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arcas_actions_msgs
  arcas_msgs
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_actions_msgs arcas_msgs roscpp
#  DEPENDS
)

add_executable(joint_controller src/joint_controller.cpp)
add_dependencies(joint_controller arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(joint_controller
         ${catkin_LIBRARIES})
add_executable(joint_parameters src/joint_parameters.cpp)
add_dependencies(joint_parameters arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(joint_parameters
         ${catkin_LIBRARIES})
