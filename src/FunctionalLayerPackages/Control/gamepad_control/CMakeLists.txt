cmake_minimum_required(VERSION 2.8.3)
project(gamepad_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  motion_action_handler
  arcas_actions_msgs
  geometry_msgs
  roscpp
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS motion_action_handler arcas_actions_msgs arcas_msgs roscpp
#  DEPENDS system
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${motion_action_handler_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")

add_library(joystick src/joystick/joystick.cpp)

add_executable(gamepad_control_node src/gamepad_control_node.cpp)
add_dependencies(gamepad_control_node arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(gamepad_control_node ${catkin_LIBRARIES} joystick)

add_executable(arm_gamepad_control_node src/arm_gamepad_control_node.cpp)
add_dependencies(arm_gamepad_control_node arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(arm_gamepad_control_node ${catkin_LIBRARIES} joystick)
