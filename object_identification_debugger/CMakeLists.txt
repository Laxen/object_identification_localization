cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(object_identification_debugger)

find_package(PCL 1.7 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (object_identification_debugger object_identification_debugger.cpp)
add_library (Identification_Class ../Headers/identification_class.cpp)
add_library (View_graph ../Headers/View_graph.cpp)
add_library (Access_Results ../Headers/access_results.cpp)
add_library (Access_Model_Data ../Headers/access_model_data.cpp)


target_link_libraries (object_identification_debugger ${PCL_LIBRARIES})
target_link_libraries (object_identification_debugger Identification_Class)
target_link_libraries (object_identification_debugger View_graph)
target_link_libraries (object_identification_debugger Access_Results ${PCL_LIBRARIES})
target_link_libraries (object_identification_debugger Access_Model_Data)






