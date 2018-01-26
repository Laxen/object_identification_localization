set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(feature_visualizer)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (feature_visualizer feature_visualizer.cpp)
target_link_libraries (feature_visualizer ${PCL_LIBRARIES})
