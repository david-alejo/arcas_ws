cmake_minimum_required(VERSION 2.8.3)
project(ual)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arcas_actions_msgs
  arcas_msgs
  roscpp
  actionlib
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_actions_msgs arcas_msgs roscpp
  DEPENDS gazebo_ros
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
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(hectorGazeboCommunication src/hectorgazebocommunication.cpp)
add_dependencies(hectorGazeboCommunication arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
add_library(simulinkGazeboCommunication src/simulinkgazebocommunication.cpp)
add_dependencies(simulinkGazeboCommunication arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
add_library(qnxCommunication src/qnxcommunication.cpp)
add_dependencies(qnxCommunication arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
add_library(ros_communication src/roscommunication.cpp)
add_dependencies(ros_communication arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
add_library(ual_hector_gazebo src/ualhectorgazebo.cpp)
add_dependencies(ual_hector_gazebo arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(ual_hector_gazebo
         hectorGazeboCommunication
         ros_communication)

add_library(ual_simulink_gazebo src/ualsimulinkgazebo.cpp)
target_link_libraries(ual_simulink_gazebo
         simulinkGazeboCommunication
         ros_communication)

add_library(ual_qnx_func src/ualqnx.cpp)
target_link_libraries(ual_qnx_func
         qnxCommunication
         ros_communication)

add_executable(ual_gazebo src/Main.cpp)
target_link_libraries(ual_gazebo
         ual_hector_gazebo
         ${catkin_LIBRARIES}
         ${GAZEBO_LIBRARIES})

add_executable(ual_simulink src/Main_simulink.cpp)
target_link_libraries(ual_simulink
         ual_simulink_gazebo
         ${catkin_LIBRARIES}
         ${GAZEBO_LIBRARIES})

add_executable(ual_qnx src/Main_qnx.cpp)
target_link_libraries(ual_qnx ${catkin_LIBRARIES} ual_qnx_func)
