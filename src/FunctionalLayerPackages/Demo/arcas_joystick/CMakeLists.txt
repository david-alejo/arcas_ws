cmake_minimum_required(VERSION 2.8.3)
project(arcas_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  arcas_msgs
  arcas_actions_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_msgs roscpp
#  DEPENDS
)

###########
## Build ##
###########
add_executable(arcas_joystick src/arcas_joystick.cpp src/joystick/joystick.cpp)
add_dependencies(arcas_joystick arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(arcas_joystick
         ${catkin_LIBRARIES})
