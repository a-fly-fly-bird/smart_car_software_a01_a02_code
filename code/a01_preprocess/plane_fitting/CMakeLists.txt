cmake_minimum_required(VERSION 3.0.0)

project(pcl_try VERSION 0.1.0)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} )

find_package(Boost REQUIRED COMPONENTS thread filesystem)

add_executable(line_ransac line_ransac.cpp)
target_link_libraries(line_ransac
			${PCL_LIBRARIES} ${Boost_LIBRARIES})
