cmake_minimum_required(VERSION 2.8.3)
project(wall_following)

add_compile_options(-std=c++11)

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wall_following
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(wall-follow
  src/WallDetect.cpp
)
target_link_libraries(wall-follow
  ${catkin_LIBRARIES}
)

add_executable(wall_detect
  src/wall_detect.cpp
  src/WallDetect.cpp
)
target_link_libraries(wall_detect
  yaml-cpp
  wall-follow
  ${catkin_LIBRARIES}
)
