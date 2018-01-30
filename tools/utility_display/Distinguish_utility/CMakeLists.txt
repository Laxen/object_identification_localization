cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(distinguish_utility_display)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (distinguish_utility_display distinguish_utility_display.cpp)
add_library (View_Graph ../../../../Headers/view_graph.cpp)
add_library (Access_Model_Data ../../../../Headers/access_model_data.cpp)


target_link_libraries (distinguish_utility_display ${PCL_LIBRARIES})
target_link_libraries (distinguish_utility_display View_Graph ${PCL_LIBRARIES})
target_link_libraries (distinguish_utility_display Access_Model_Data ${PCL_LIBRARIES})






