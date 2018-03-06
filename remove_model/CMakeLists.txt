cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(remove_model)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (remove_model remove_model.cpp)
add_library (Access_Model_Data ../headers/access_model_data.cpp)

target_link_libraries (remove_model ${PCL_LIBRARIES})
target_link_libraries (remove_model Access_Model_Data)






