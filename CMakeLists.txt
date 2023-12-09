cmake_minimum_required(VERSION 3.22.1)

project(v5-devices-sim)

add_executable(test src/main.cpp)
add_library(${PROJECT_NAME} src/main.cpp)