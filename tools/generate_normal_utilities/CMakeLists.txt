cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_utility_test)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (normal_utility_test normal_utility_test.cpp)
add_library (Normal_Utility ../../headers/normal_utility.cpp)
add_library (View_Graph ../../headers/view_graph.cpp)
add_library (Access_Model_Data ../../headers/access_model_data.cpp)


target_link_libraries (normal_utility_test ${PCL_LIBRARIES})
target_link_libraries (normal_utility_test Normal_Utility ${PCL_LIBRARIES})
target_link_libraries (normal_utility_test View_Graph ${PCL_LIBRARIES})
target_link_libraries (normal_utility_test Access_Model_Data ${PCL_LIBRARIES})






