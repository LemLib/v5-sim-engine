cmake_minimum_required(VERSION 3.22.1)
project(v5_sim_engine)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/build/bin>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/build/bin>")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/build/bin>")

add_subdirectory(pros-sim)
add_executable(v5_sim_engine src/main.cpp)

target_include_directories(v5_sim_engine PUBLIC pros-sim/simulator/include)
target_link_libraries(v5_sim_engine v5_sim_pros)