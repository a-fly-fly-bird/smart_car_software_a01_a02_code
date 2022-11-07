cmake_minimum_required(VERSION 3.0.2)
project(a02_task3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ugv_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES a02_task3
  CATKIN_DEPENDS roscpp rospy ugv_sdk
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(read_info src/read_info.cpp)
target_link_libraries(read_info ${catkin_LIBRARIES})
