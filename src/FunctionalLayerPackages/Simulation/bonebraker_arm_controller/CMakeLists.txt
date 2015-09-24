cmake_minimum_required(VERSION 2.8.3)
project(bonebraker_arm_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arcas_msgs
  roscpp
  sensor_msgs
  geometry_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_msgs roscpp
  DEPENDS gazebo_ros
)

message(MAMONEO)
message(${catkin_INCLUDE_DIRS})
message(${GAZEBO_INCLUDE_DIRS})
message(fin MAMONEO)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(joint_control src/joint_control.cpp)
add_dependencies(joint_control arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
add_library(bonebraker_arm_controller src/bonebraker_arm_controller.cpp)
add_dependencies(bonebraker_arm_controller arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)

target_link_libraries(bonebraker_arm_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} joint_control)
