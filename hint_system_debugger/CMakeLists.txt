cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(hint_system_debugger)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (hint_system_debugger hint_system_debugger.cpp)
add_library (Hint_system_class ../Headers/Hint_system_class.cpp)
add_library (View_graph ../Headers/View_graph.cpp)
add_library (Access_Model_Data ../Headers/access_model_data.cpp)
add_library (Access_Results ../Headers/Access_Results.cpp)


target_link_libraries (hint_system_debugger ${PCL_LIBRARIES})
target_link_libraries (hint_system_debugger Hint_system_class ${PCL_LIBRARIES})
target_link_libraries (hint_system_debugger Access_Results ${PCL_LIBRARIES})
target_link_libraries (hint_system_debugger View_graph ${PCL_LIBRARIES})
target_link_libraries (hint_system_debugger Access_Model_Data ${PCL_LIBRARIES})






