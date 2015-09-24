cmake_minimum_required(VERSION 2.8.3)
project(aal)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arcas_actions_msgs
  arcas_msgs
  roscpp
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_actions_msgs arcas_msgs roscpp
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

FILE(GLOB_RECURSE HDRS "include/aal/*.h")
include_directories(include src/brazo_ros_ert_rtw/)

# Add matlab c libraries
FILE(GLOB srcs_c "${PROJECT_SOURCE_DIR}/src/brazo_ros_ert_rtw/*.cpp")
add_library(matlab_c ${srcs_c})
add_dependencies(matlab_c arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)

add_library(aal_gaz
      src/arm_abstraction_layer_gazebo.cpp
      src/aal_extension_action.cpp
      ${HDRS})
add_dependencies(aal_gaz arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(aal_gaz ${catkin_LIBRARIES} matlab_c)

add_library(aal_sim
      src/arm_abstraction_layer_simulink.cpp
      src/aal_extension_action.cpp
      ${HDRS})
add_dependencies(aal_sim arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(aal_sim ${catkin_LIBRARIES} matlab_c)

add_library(aal_qnx_lib
      src/arm_abstraction_layer_qnx.cpp
      src/aal_extension_action_qnx.cpp
      ${HDRS})
add_dependencies(aal_qnx_lib arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(aal_qnx_lib ${catkin_LIBRARIES})

add_executable(aal_gazebo src/main.cpp)
add_dependencies(aal_gazebo arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(aal_gazebo aal_gaz)

add_executable(aal_simulink src/main_simulink.cpp)
add_dependencies(aal_simulink arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(aal_simulink aal_sim)

add_executable(aal_qnx src/main_qnx.cpp)
add_dependencies(aal_qnx arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(aal_qnx aal_qnx_lib)

