cmake_minimum_required(VERSION 2.8.3)
project(HACD_Lib)

find_package(catkin REQUIRED)

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
 INCLUDE_DIRS include
 LIBRARIES HACD_Lib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include/HACD_Lib include)

set(PROJECT_INC_FILES include/HACD_Lib/hacdCircularList.h include/HACD_Lib/hacdGraph.h include/HACD_Lib/hacdICHull.h include/HACD_Lib/hacdMeshDecimator.h include/HACD_Lib/hacdRaycastMesh.h include/HACD_Lib/hacdVector.h    include/HACD_Lib/hacdVersion.h include/HACD_Lib/hacdCircularList.inl  include/HACD_Lib/hacdHACD.h  include/HACD_Lib/hacdManifoldMesh.h  include/HACD_Lib/hacdMicroAllocator.h  include/HACD_Lib/hacdSArray.h  include/HACD_Lib/hacdVector.inl)

set(PROJECT_CPP_FILES src/hacdGraph.cpp src/hacdHACD.cpp src/hacdICHull.cpp src/hacdManifoldMesh.cpp src/hacdMeshDecimator.cpp src/hacdMicroAllocator.cpp   src/hacdRaycastMesh.cpp )

add_library(${PROJECT_NAME} ${PROJECT_CPP_FILES} ${PROJECT_INC_FILES})

