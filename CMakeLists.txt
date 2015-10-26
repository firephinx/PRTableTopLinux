CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
endif()

IF(NOT DEFINED CMAKE_BUILD_TYPE)
  # No effect for multi-configuration generators (e.g. for Visual Studio)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF()

PROJECT(PRTabletop)

SET(MY_DIR ${PRTabletop_SOURCE_DIR})
SET(DEPENDS_DIR "${MY_DIR}/../../depends" CACHE STRING "Dependency directory")
SET(INCLUDE_DIR ${MY_DIR}/include)

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

# The example build system is standalone and will work out-of-tree with these files copied
SET(freenect2_ROOT_DIR ${MY_DIR}/../..)
SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h

FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
FIND_PACKAGE(PCL 1.2 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${freenect2_ROOT_DIR}/cmake_modules) # FindGLFW3.cmake

IF(TARGET freenect2)
  MESSAGE(STATUS "Using in-tree freenect2 target")
  SET(freenect2_LIBRARIES freenect2)
ELSE()
  FIND_PACKAGE(freenect2 REQUIRED)
ENDIF()

INCLUDE_DIRECTORIES(
  ${INCLUDE_DIR}
  ${freenect2_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

SET(SOURCES
  include/calib.h
  #include/entity.h
  #include/entity.pb.h
  include/objectSegmentor.h

  include/opencv.h
  include/pcl.h

  #include/point2D.pb.h
  #include/pose2D.pb.h

  include/settings.h
  #include/tcp.h
  include/timer.h
  include/utilities.h

  src/calib.cpp
  #src/entity.cpp
  #src/entity.pb.cc
  src/objectSegmentor.cpp
  #src/point2D.pb.cc
  #src/pose2D.pb.cc
  #src/tcp.cpp
  src/timer.cpp
  src/utilities.cpp
)

LINK_DIRECTORIES(
  ${freenect2_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

ADD_DEFINITIONS(${PCL_DEFINITIONS})

SET(EyekinMain
  eyeKinMain.cpp
)

SET(PRTabletop_LIBRARIES
  ${freenect2_LIBRARIES}
)
LIST(APPEND PRTabletop_LIBRARIES
  ${OpenCV_LIBS}
)

IF(ENABLE_OPENGL)
  FIND_PACKAGE(GLFW3)
  FIND_PACKAGE(OpenGL)
  IF(GLFW3_FOUND AND OPENGL_FOUND)
    INCLUDE_DIRECTORIES(
      ${GLFW3_INCLUDE_DIRS}
      ${flextGL_INCLUDE_DIRS}
    )

    LINK_DIRECTORIES(${GLFW3_LIBRARY_DIRS})
    LIST(APPEND EyekinMain
      viewer.cpp
      ${SOURCES}
      ${flextGL_SOURCES}
    )
    LIST(APPEND PRTabletop_LIBRARIES
      ${GLFW3_LIBRARIES}
      ${OPENGL_gl_LIBRARY}
    )
    ADD_DEFINITIONS(-DEXAMPLES_WITH_OPENGL_SUPPORT=1)
  ENDIF()
ENDIF(ENABLE_OPENGL)

ADD_EXECUTABLE(PRTabletop
  ${EyekinMain}
)

TARGET_LINK_LIBRARIES(PRTabletop
  ${PRTabletop_LIBRARIES}
)
