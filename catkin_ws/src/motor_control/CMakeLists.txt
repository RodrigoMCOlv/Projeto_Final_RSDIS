cmake_minimum_required(VERSION 3.0.2)
project(motor_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-Wall -Wuseless-cast -Wformat-nonliteral)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_executable(motor_control_node src/motor_control_node.cpp)

target_link_libraries(motor_control_node
    ${catkin_LIBRARIES}
    -lwiringPi
    -lpthread
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(motor_control_node ${catkin_EXPORTED_TARGETS})