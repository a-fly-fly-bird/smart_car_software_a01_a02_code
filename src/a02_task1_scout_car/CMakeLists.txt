cmake_minimum_required(VERSION 3.0.2)
project(a02_scout_car_task_1)

find_package( OpenCV  REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread filesystem)

find_package(PCL REQUIRED) # This includes all modules

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  OpenCV
  pcl_conversions
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES a02_scout_car_task_1
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_conversions pcl_ros
#  DEPENDS system_lib
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(imu_subscriber src/imu_subscriber.cpp)
target_link_libraries(imu_subscriber ${catkin_LIBRARIES})

add_executable(image_viewer src/image_viewer.cpp)
target_link_libraries(image_viewer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pcl_viewer src/pcl_viewer.cpp)
target_link_libraries(pcl_viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(depth_image_viewer src/depth_image_viewer.cpp)
target_link_libraries(depth_image_viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(distance_measurement src/distance_measurement.cpp)
target_link_libraries(distance_measurement algorithm_set ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
