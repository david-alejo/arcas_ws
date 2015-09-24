SET(HEADERS ArgumentData.h DegMinSec.h functions.h HessianPlane.h Point3D.h RealVector.h RandomNumberGenerator.h FormattedTime.h)

SET(FUN_HEADERS src/ArgumentData.h src/DegMinSec.h src/functions.h
    src/HessianPlane.h src/Point3D.h src/RealVector.h src/RandomNumberGenerator.h src/FormattedTime.h PARENT_SCOPE)

SET(FUN_SOURCE src/ArgumentData.cpp src/DegMinSec.cpp src/functions.cpp
    src/HessianPlane.cpp src/Point3D.cpp src/RealVector.cpp src/RandomNumberGenerator.cpp src/FormattedTime.cpp PARENT_SCOPE)

## Multiplatform stuff
SET(INSTALL_LOCATION /usr/local/)
SET(BUILD_TYPE SHARED)
if (WIN32)
  SET(INSTALL_LOCATION C:/QtSDK/mingw/)
  SET(BUILD_TYPE STATIC)
endif (WIN32)

INSTALL(FILES  ${HEADERS}
        DESTINATION ${INSTALL_LOCATION}include/functions
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ) 
