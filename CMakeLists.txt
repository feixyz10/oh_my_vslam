cmake_minimum_required(VERSION 3.10)
project(oh_my_vslam)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

message("${CMAKE_CURRENT_SOURCE_DIR}/build/lib")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  sensor_msgs
)

find_package(g3log REQUIRED SYSTEM)
find_package(PCL QUIET)
find_package(GTest REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${G3LOG_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(common)
add_subdirectory(oh_my_vslam)
