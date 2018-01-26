cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(save_local_features)

find_package(PCL 1.7 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (save_local_features save_local_features.cpp)
add_library (Access_Model_data ../../../Headers/Access_Model_data.cpp)

target_link_libraries (save_local_features ${PCL_LIBRARIES})
target_link_libraries (save_local_features Access_Model_data ${PCL_LIBRARIES})






