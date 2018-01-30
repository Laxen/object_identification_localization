cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CURL_LIBRARY "-lcurl") 

project(background_creator)

find_package(PCL 1.8 REQUIRED)
find_package(CURL REQUIRED) 

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CURL_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(Access_Results ../../headers/access_results.cpp)
add_library(Config_Reader ../../headers/config_reader.cpp)
add_library(Manipulation ../../headers/manipulation.cpp)
add_library(Robot_Data_Fetcher ../../headers/robot_data_fetcher.cpp)

add_executable (background_creator background_creator.cpp)
target_link_libraries (background_creator Access_Results Config_Reader Manipulation Robot_Data_Fetcher ${PCL_LIBRARIES} ${CURL_LIBRARY})
