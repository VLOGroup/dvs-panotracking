PROJECT(dvs_panotracking)

cmake_minimum_required(VERSION 2.8)
FILE(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" OT_CMAKE_MODULE_PATH)
SET(CMAKE_MODULE_PATH ${OT_CMAKE_MODULE_PATH})

set(CMAKE_BUILD_TYPE Release)


##-----------------------------------------------------------------------------
# ImageUtilities
#change the following line to whatever graphics card you have
set(ImageUtilities_DIR $ENV{IMAGEUTILITIES_ROOT})
set(IMAGEUTILITIES_PREFER_STATIC_LIBRARIES false)
find_package(ImageUtilities REQUIRED COMPONENTS iucore iuio iumath iugui)
cuda_include_directories(${IMAGEUTILITIES_INCLUDE_DIR})
include_directories(${IMAGEUTILITIES_INCLUDE_DIR})

##-----------------------------------------------------------------------------
## Qt5
set(CMAKE_AUTOMOC ON)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5OpenGL)
qt5_add_resources(UI_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/resources/resources.qrc)
##-----------------------------------------------------------------------------
## Eigen
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/include/eigen3)
## Compiler Flags
if(WIN32)
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NODEFAULTLIB:LIBCMT.lib /MDd")
endif(WIN32)
add_definitions("-std=c++11 -fpermissive -O3 -DPARALLEL -ffast-math")

SET(CUDA_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/common.cu
  ${CMAKE_CURRENT_SOURCE_DIR}/direct.cu
  ${CMAKE_CURRENT_SOURCE_DIR}/common.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/parameters.cpp)
SET(HEADER_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/event.h
  ${CMAKE_CURRENT_SOURCE_DIR}/scopedtimer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/common.h
  ${CMAKE_CURRENT_SOURCE_DIR}/common.cuh
  ${CMAKE_CURRENT_SOURCE_DIR}/direct.cuh
  ${CMAKE_CURRENT_SOURCE_DIR}/parameters.h)

if(WIN32)
  cuda_add_library(dvs-tracking-common  ${CUDA_FILES})
else(WIN32)
  cuda_add_library(dvs-tracking-common STATIC ${CUDA_FILES})
  target_link_libraries(dvs-tracking-common ${IMAGEUTILITIES_LIBRARIES})
endif(WIN32)
target_link_libraries(dvs-tracking-common ${OpenCV_LIBRARIES} ${OpenCV_LIBS} cnpy)

SET ( GUI_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/live_tracking_gui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/trackingmainwindow.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/dvscameraworker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/trackingworker.cpp)

link_directories(/usr/local/lib/x86_64-linux-gnu/) # libcaer
link_directories(/usr/local/lib64/)
CUDA_ADD_EXECUTABLE(live_tracking_gui ${GUI_FILES} ${HEADER_FILES} ${UI_RESOURCES})
TARGET_LINK_LIBRARIES(live_tracking_gui dvs-tracking-common X11 Qt5::Widgets Qt5::OpenGL caer)


