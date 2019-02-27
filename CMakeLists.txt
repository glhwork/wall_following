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
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main
  src/main.cpp
)
target_link_libraries(main
  yaml-cpp
  ${catkin_LIBRARIES}
)