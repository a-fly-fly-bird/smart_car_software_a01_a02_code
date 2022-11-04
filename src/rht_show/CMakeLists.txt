cmake_minimum_required(VERSION 3.0.2)
project(rht_show)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_learning_pkg
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
 

add_executable( image_show  
src/rht_show.cpp)
 

target_link_libraries(image_show ${catkin_LIBRARIES} ${OpenCV_LIBS})
