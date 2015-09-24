cmake_minimum_required(VERSION 2.8.3)
project(bonebraker_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  arcas_msgs
  sensor_msgs
  hector_gazebo_plugins
  hector_uav_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
  CATKIN_DEPENDS arcas_msgs roscpp
  DEPENDS gazebo_ros hector_gazebo_plugins
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#Include directories for header files
include_directories (${PROJECT_SOURCE_DIR}/src/ext_mode ${PROJECT_SOURCE_DIR}/src/quadrotor_controller_ert_rtw)
add_definitions( -DCLASSIC_INTERFACE=1 -DGLNXA32  -O -fPIC -DEXT_MODE=1 -DHAVESTDIO -DINTEGER_CODE=0 -DMAT_FILE=0 -DMODEL=quadrotor_controller -DMT=0 -DNCSTATES=0 -DNUMST -DON_TARGET_WAIT_FOR_START=0 -DONESTEPFNC=1 -DTERMFCN=1 -DTID01EQ=0 -DUNIX )

add_library(hector_gazebo_ros_baro src/gazebo_ros_baro.cpp)
add_dependencies(hector_gazebo_ros_baro arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)

FILE(GLOB ext_mode src/ext_mode/*.c)
add_library(matlab_ext_mode ${ext_mode})
add_dependencies(matlab_ext_mode arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)

FILE(GLOB model_library src/quadrotor_controller_ert_rtw/*.cpp)
add_library(matlab_model ${model_library})
add_dependencies(matlab_model arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)

add_library(bonebraker_controller src/bonebraker_controller.cpp)
target_link_libraries(bonebraker_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} matlab_ext_mode matlab_model)

