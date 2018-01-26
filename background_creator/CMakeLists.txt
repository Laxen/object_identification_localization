cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(background_creator)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(Access_Results ../../Headers/Access_Results.cpp)

add_executable (background_creator background_creator.cpp)
target_link_libraries (background_creator Access_Results ${PCL_LIBRARIES})
