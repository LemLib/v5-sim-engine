cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 23)

project(v5_device_sim)
add_library(v5_device_sim SHARED src/main.cpp)
target_include_directories(v5_device_sim PUBLIC include)