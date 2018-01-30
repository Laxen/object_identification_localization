cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(offline_data_generation)

find_package(PCL 1.8 REQUIRED)

#set(CMAKE_BUILD_TYPE Debug)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (offline_data_generation offline_data_generation.cpp)
add_library (Render_Synthetic_Views ../headers/render_synthetic_views.cpp)
add_library (View_Graph ../headers/view_graph.cpp)
add_library (Access_Model_Data ../headers/access_model_data.cpp)
add_library (Render_Views_Tesselated_Sphere_Modified ../headers/render_views_tesselated_sphere_modified.cpp)
add_library (View_Feature_Score ../headers/view_feature_score.cpp)
add_library (Config_Reader ../headers/config_reader.cpp)


target_link_libraries (offline_data_generation ${PCL_LIBRARIES})
target_link_libraries (offline_data_generation Render_Synthetic_Views ${PCL_LIBRARIES})
target_link_libraries (offline_data_generation View_Graph)
target_link_libraries (offline_data_generation Access_Model_Data)
target_link_libraries (offline_data_generation Render_Views_Tesselated_Sphere_Modified)
target_link_libraries (offline_data_generation View_Feature_Score)
target_link_libraries (offline_data_generation Config_Reader)






