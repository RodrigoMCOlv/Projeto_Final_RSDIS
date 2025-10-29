cmake_minimum_required(VERSION 3.0.2)
project(line_statistics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(statistics_node src/statistics_node.cpp)
target_link_libraries(statistics_node ${catkin_LIBRARIES})

install(TARGETS statistics_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
