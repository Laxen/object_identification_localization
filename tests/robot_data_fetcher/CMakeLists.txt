set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "")

cmake_minimum_required(VERSION 2.8)

set(CURL_LIBRARY "-lcurl") 

find_package(CURL REQUIRED) 

add_library(Robot_Data_Fetcher ../../Headers/robot_data_fetcher.cpp)

include_directories(${CURL_INCLUDE_DIR})

add_executable (robot_data_fetcher robot_data_fetcher.cpp)
target_link_libraries(robot_data_fetcher Robot_Data_Fetcher ${CURL_LIBRARIES})
