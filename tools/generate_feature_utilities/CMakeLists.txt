cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(generate_feature_utilities)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(Access_Model_Data ../../../Headers/access_model_data.cpp)
add_library(View_Graph ../../../Headers/view_graph.cpp)
add_library(View_Feature_Score ../../../Headers/view_feature_score.cpp)

add_executable (generate_feature_utilities generate_feature_utilities.cpp)
target_link_libraries (generate_feature_utilities ${PCL_LIBRARIES})
target_link_libraries (generate_feature_utilities Access_Model_Data ${PCL_LIBRARIES})
target_link_libraries (generate_feature_utilities View_Graph)
target_link_libraries (generate_feature_utilities View_Feature_Score)
