cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navigation
#  CATKIN_DEPENDS move_base roscpp
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

