set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(camera_hand_calibration)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (camera_hand_calibration camera_hand_calibration.cpp)
add_library(Access_Results ../Headers/Access_Results.cpp)

target_link_libraries (camera_hand_calibration Access_Results ${PCL_LIBRARIES})
