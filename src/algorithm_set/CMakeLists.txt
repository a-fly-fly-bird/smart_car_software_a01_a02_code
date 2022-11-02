cmake_minimum_required(VERSION 3.0.2)
project(algorithm_set)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES algorithm_set
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(algorithm_set STATIC 
    src/distance.cpp
)

target_include_directories(algorithm_set
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)
