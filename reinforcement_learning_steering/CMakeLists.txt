cmake_minimum_required(VERSION 3.10.0)
project(reinforcement_learning_steering)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set (CMAKE_CXX_STANDARD 14)

find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
    Gui
    PrintSupport
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
)

add_message_files(
    FILES
    RLEpisodeState.msg
    RLSimulation.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Printer_INCLUDE_DIRS}
)

add_definitions(
    ${Qt5Widgets_DEFINITIONS}
    ${Qt5Printer_DEFINITIONS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}  -std=c++11 -Wall")

add_library(libQCustomPlot 
    src/rl_plot/qcustomplot.cpp 
    src/rl_plot/moc_qcustomplot.cpp)

target_link_libraries(libQCustomPlot
    Qt5::Gui
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)

set_target_properties(libQCustomPlot
    PROPERTIES OUTPUT_NAME QCustomPlot
)

target_include_directories(libQCustomPlot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(rl_node 
    src/rl_node.cpp  
    include/reinforcement_learning_steering/RLMainWindow.h
    src/rl_plot/RLMainWindow.cpp 
    src/rl_plot/mainwindow.ui)

target_link_libraries(rl_node 
    Qt5::Gui
    Qt5::Core
    Qt5::Widgets
    libQCustomPlot
    ${catkin_LIBRARIES})

