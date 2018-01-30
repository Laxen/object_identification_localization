cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#project(visual_ex)
project(generate_new_distinguish_utilities)

find_package(PCL 1.7 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (generate_new_distinguish_utilities generate_new_distinguish_utilities.cpp)
add_library (Similar_Object_Recognition ../../Headers/similar_object_recognition.cpp)
add_library (Access_Model_Data ../../Headers/access_model_data.cpp)

target_link_libraries (generate_new_distinguish_utilities ${PCL_LIBRARIES})
target_link_libraries (generate_new_distinguish_utilities Similar_Object_Recognition)
target_link_libraries (generate_new_distinguish_utilities Access_Model_Data ${PCL_LIBRARIES})
#add_executable (visual_ex visual_ex.cpp)
#target_link_libraries (visual_ex ${PCL_LIBRARIES})
