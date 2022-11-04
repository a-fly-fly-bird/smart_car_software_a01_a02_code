cmake_minimum_required(VERSION 3.0.0)

project(pcl_try VERSION 0.1.0)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} )

find_package(Boost REQUIRED COMPONENTS thread filesystem)

add_executable(pass_through pass_through.cpp)
target_link_libraries(pass_through 
			${PCL_LIBRARIES} ${Boost_LIBRARIES})


add_executable(statistics_through statistics_through.cpp)
target_link_libraries(statistics_through 
			${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ramdom_sampling ramdom_sampling.cpp)
target_link_libraries(ramdom_sampling 
			${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(voxel_grid_sampling voxel_grid_sampling.cpp)
target_link_libraries(voxel_grid_sampling 
			${PCL_LIBRARIES} ${Boost_LIBRARIES})