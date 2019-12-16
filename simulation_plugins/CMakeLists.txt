cmake_minimum_required(VERSION 2.8.3)
project(simulation_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  simulations_messages
)

find_package(gazebo REQUIRED)
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package (Qt5Test REQUIRED)
find_package(Protobuf REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(
  include
  ${Boost_INLUCDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Test_LIBRARIES}
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES gazebo_sim
 CATKIN_DEPENDS roscpp simulations_messages
)

add_library(add_force SHARED src/DoublePendulumPlugin.cpp)
target_link_libraries(add_force ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${roscpp_LIBRARIES})


