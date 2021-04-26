cmake_minimum_required(VERSION 3.10)
project(oh_my_vslam)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(g3log REQUIRED SYSTEM)
find_package(yaml-cpp REQUIRED)
find_package(PCL QUIET)
find_package(GTest REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

set(THIRD_PARTY_LIBS
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  ${GTEST_BOTH_LIBRARIES}
  ${G3LOG_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM
  ${G3LOG_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(common)
add_subdirectory(oh_my_vslam)
