cmake_minimum_required(VERSION 2.8.3)
project(com_xbee)

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

add_executable(xbee src/xbee.cpp)

target_link_libraries(xbee
  ${catkin_LIBRARIES}
)

