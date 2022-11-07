cmake_minimum_required(VERSION 3.0.2)
project(a01_image_process)

find_package( OpenCV  REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
  cv_bridge
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a01_image_process
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(gray src/gray.cpp)
target_link_libraries(gray ${OpenCV_LIBS})

add_executable(ros_gray src/ros_gray.cpp)
target_link_libraries(ros_gray ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(blur src/blur.cpp)
target_link_libraries(blur ${OpenCV_LIBS})

add_executable(ros_blur src/ros_blur.cpp)
target_link_libraries(ros_blur ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(fast_feature_detector src/fast_feature_detector.cpp)
target_link_libraries(fast_feature_detector ${OpenCV_LIBS})

add_executable(ros_fast_feature_detector src/ros_fast_feature_detector.cpp)
target_link_libraries(ros_fast_feature_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})