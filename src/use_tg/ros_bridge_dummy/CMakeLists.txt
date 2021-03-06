cmake_minimum_required(VERSION 2.8.3)
project(ros_bridge_dummy)

find_package(catkin REQUIRED COMPONENTS functions roscpp std_msgs geometry_msgs arcas_msgs arcas_actions_msgs actionlib)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
#  INCLUDE_DIRS 
 LIBRARIES ros_bridge_dummy
 CATKIN_DEPENDS functions roscpp std_msgs geometry_msgs arcas_msgs arcas_actions_msgs actionlib
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/util.cpp)
add_dependencies(${PROJECT_NAME} arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ros_bridge_node src/ROSBridgeDummy.cpp)
target_link_libraries(ros_bridge_node functions ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(ros_bridge_node_incremental src/ROSBridgeDummyIncremental.cpp)
target_link_libraries(ros_bridge_node_incremental functions ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(arm_commander src/arm_commander.cpp)
target_link_libraries(arm_commander functions ${catkin_LIBRARIES} ${PROJECT_NAME} ${Boost_INCLUDE_DIRS})

