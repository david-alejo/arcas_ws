cmake_minimum_required(VERSION 2.8.3)
project(tg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS functions roscpp std_msgs geometry_msgs arcas_msgs arcas_actions_msgs actionlib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
 INCLUDE_DIRS include
 LIBRARIES tg
 CATKIN_DEPENDS functions roscpp std_msgs geometry_msgs arcas_msgs arcas_actions_msgs actionlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME} src/ContinousTrajectoryFollower.cpp src/DiscreteTrajectoryFollower.cpp)
add_dependencies(${PROJECT_NAME} arcas_msgs_generate_messages_cpp arcas_actions_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} qwt)

## Declare a cpp executable
add_executable(trajectory_generator src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator ${catkin_LIBRARIES} ${PROJECT_NAME})

