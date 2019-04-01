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
  src/scan_based/WallDetect.cpp
  src/scan_based/MoveSide.cpp
)
target_link_libraries(wall-follow
  ${catkin_LIBRARIES}
)

# ===== ELF -> scan_based =====

add_executable(wall_detect
  src/scan_based/wall_detect.cpp
  src/scan_based/WallDetect.cpp
)
target_link_libraries(wall_detect
  yaml-cpp
  wall-follow
  ${catkin_LIBRARIES}
)

add_executable(move_side
  src/scan_based/move_side.cpp
  src/scan_based/MoveSide.cpp
)
target_link_libraries(move_side
  yaml-cpp
  wall-follow
  ${catkin_LIBRARIES}
)

add_executable(test_map
  src/scan_based/test_map.cpp
)
target_link_libraries(test_map
  yaml-cpp
  ${catkin_LIBRARIES}
)

# ===== ELF -> map_based =====
add_library(detection
  src/map_based/Detection.cpp
)
target_link_libraries(detection
  ${catkin_LIBRARIES}
)

add_executable(detect
  src/map_based/detect.cpp
)
target_link_libraries(detect
  yaml-cpp
  detection
  ${catkin_LIBRARIES}
)