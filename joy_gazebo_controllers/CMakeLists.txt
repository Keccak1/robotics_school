cmake_minimum_required(VERSION 2.8.3)
project(joy_gazebo_controllers)

find_package(catkin REQUIRED COMPONENTS
  rospy
  joy
  simulations_messages
)


catkin_package(
 CATKIN_DEPENDS rospy joy simulations_messages
)


