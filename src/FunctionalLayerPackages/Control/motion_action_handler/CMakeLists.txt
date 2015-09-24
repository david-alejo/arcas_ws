cmake_minimum_required(VERSION 2.8.3)
project(motion_action_handler)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arcas_module_main
  arcas_actions_msgs
  arcas_msgs
  std_msgs
  roscpp
  tf
  actionlib
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_module_main arcas_actions_msgs arcas_msgs roscpp
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${arcas_module_main_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(motion_action_handler src/motionactionhandler.cpp)
add_dependencies(motion_action_handler arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(motion_action_handler
         ${catkin_LIBRARIES}
         )

add_executable(motion_action_handler_node src/Main.cpp)
add_dependencies(motion_action_handler_node arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(motion_action_handler_node
         ${catkin_LIBRARIES}
         motion_action_handler
         )
