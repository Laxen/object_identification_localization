cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(measure_distance)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (measure_distance measure_distance.cpp)
target_link_libraries (measure_distance ${PCL_LIBRARIES})
