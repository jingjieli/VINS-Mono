set(PROJECT_NAME open_chisel)
cmake_minimum_required(VERSION 2.8.3)
project(${PROJECT_NAME})
 
#if (DEFINED CATKIN_TOPLEVEL OR (NOT ("$ENV{ROS_DISTRO}" STREQUAL "fuerte")))
#    include(${PROJECT_SOURCE_DIR}/catkin.cmake)
#else ()
#    include(${PROJECT_SOURCE_DIR}/rosbuild.cmake)
#endif ()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/include/
)

file(GLOB OPEN_CHISEL_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/include/*.h*
  ${PROJECT_SOURCE_DIR}/src/camera/*.cpp
  ${PROJECT_SOURCE_DIR}/src/geometry/*.cpp
  ${PROJECT_SOURCE_DIR}/src/io/*.cpp
  ${PROJECT_SOURCE_DIR}/src/marching_cubes/*.cpp
  ${PROJECT_SOURCE_DIR}/src/mesh/*.cpp
  ${PROJECT_SOURCE_DIR}/src/pointcloud/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

#set(OPEN_CHISEL_SOURCE_FILES
#  ${PROJECT_SOURCE_DIR}/src/camera/Intrinsics.cpp
#  ${PROJECT_SOURCE_DIR}/src/camera/PinholeCamera.cpp
#)

message(STATUS "${OPEN_CHISEL_SOURCE_FILES}")

add_library(${PROJECT_NAME} SHARED ${OPEN_CHISEL_SOURCE_FILES})
#target_link_libraries(${PROJECT_NAME})