cmake_minimum_required(VERSION 3.22.1)
project(v5_sim_engine)

set(TARGET "WINDOWS")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/bin>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/bin>")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/bin>")

add_subdirectory(./pros-sim)
add_executable(v5_sim_engine
        src/main.cpp
        include/units.hpp
        include/Vector2D.hpp
        src/bot.cpp
        include/bot.h
        include/util.h
        include/matrix.h
        src/math.cpp
        include/math.h
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

target_include_directories(v5_sim_engine PUBLIC pros-sim/simulator/include)
target_include_directories(v5_sim_engine PUBLIC pros-sim/pros/include)
target_include_directories(v5_sim_engine PUBLIC include)

add_dependencies(v5_sim_engine v5_sim_user)


if(${TARGET} STREQUAL "WINDOWS")
    # the name of the target operating system
    set(CMAKE_SYSTEM_NAME Windows)
    set(CMAKE_SYSTEM_VERSION Windows)

    include_directories(pros-sim/windows)
    target_link_libraries(v5_sim_engine PRIVATE v5_sim_user)
else()
    target_link_libraries(v5_sim_engine PRIVATE v5_sim_user)
endif()