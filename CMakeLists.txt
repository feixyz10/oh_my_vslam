cmake_minimum_required(VERSION 3.10)
project(oh_my_vslam)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(OpenCV REQUIRED)
# message("${OpenCV_INCLUDE_DIRS}    ${OpenCV_INCLUDE_DIRS}    ${OpenCV_LIBS}")
find_package(g3log REQUIRED SYSTEM)
find_package(PCL QUIET)
find_package(GTest REQUIRED)

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

# target_link_libraries(xxx
#   ${OpenCV_LIBS}
# )
