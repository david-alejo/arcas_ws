cmake_minimum_required(VERSION 2.8.3)
project(rviz_scenario)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ual
#  CATKIN_DEPENDS
#  DEPENDS
)

###########
## Build ##
###########

add_executable(rvizScenario src/main.cpp)
target_link_libraries(rvizScenario
         ${catkin_LIBRARIES})
