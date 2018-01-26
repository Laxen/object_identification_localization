cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_utility_test)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (normal_utility_test normal_utility_test.cpp)
add_library (Normal_utility ../../Headers/Normal_utility.cpp)
add_library (View_graph ../../Headers/View_graph.cpp)
add_library (Access_Model_data ../../Headers/Access_Model_data.cpp)


target_link_libraries (normal_utility_test ${PCL_LIBRARIES})
target_link_libraries (normal_utility_test Normal_utility ${PCL_LIBRARIES})
target_link_libraries (normal_utility_test View_graph ${PCL_LIBRARIES})
target_link_libraries (normal_utility_test Access_Model_data ${PCL_LIBRARIES})






