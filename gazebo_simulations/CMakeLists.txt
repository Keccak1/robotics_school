cmake_minimum_required(VERSION 2.8.3)
project(gazebo_simulations)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)
  
find_package(gazebo REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(
  include
  ${Boost_INLUCDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 LIBRARIES gazebo_sim
 CATKIN_DEPENDS gazbo_ros 
)
