cmake_minimum_required(VERSION 2.8.3)
project(simulations_messages)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_message_files(
  DIRECTORY
  msg/common  
)

add_message_files(
  DIRECTORY
  msg/double_pendulum
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)


