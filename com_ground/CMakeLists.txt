cmake_minimum_required(VERSION 2.8.3)
project(com_ground)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
)

catkin_package(
  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_node src/serial_node.cpp)

target_link_libraries(serial_node
  ${catkin_LIBRARIES}
)

add_executable(trajectory src/trajectory.cpp)

target_link_libraries(trajectory
  ${catkin_LIBRARIES}
)

