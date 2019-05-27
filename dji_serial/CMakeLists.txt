cmake_minimum_required(VERSION 2.8.3)
project(dji_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dji_serial_node src/dji_serial_node.cpp)

target_link_libraries(dji_serial_node
  ${catkin_LIBRARIES}
)

