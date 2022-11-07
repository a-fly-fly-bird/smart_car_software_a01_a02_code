cmake_minimum_required(VERSION 3.0.2)
project(a02_task3)

find_package(Boost REQUIRED COMPONENTS thread filesystem)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  car_basic_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a02_task3_demo
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES(/home/agilex/catkin_ws/devel/lib)
LINK_LIBRARIES(ugv_sdk)

add_executable(info_publisher src/info_publisher.cpp)
target_link_libraries(info_publisher ugv_sdk ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_install_python(PROGRAMS scripts/server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

