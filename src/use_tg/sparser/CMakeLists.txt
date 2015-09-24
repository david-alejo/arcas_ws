cmake_minimum_required(VERSION 2.8.3)
project(sparser)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include;include/sparser
 LIBRARIES sparser
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include/sparser)

## Declare a cpp library
add_library(${PROJECT_NAME} src/Checker.cpp src/ParseBlock.cpp src/ParseProperty.cpp src/Parser.cpp src/StringUtil.cpp)
target_link_libraries(${PROJECT_NAME} boost_regex )


