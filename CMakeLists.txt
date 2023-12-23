cmake_minimum_required(VERSION 3.22.1)
project(v5_sim_engine)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/bin>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/bin>")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "$<1:${CMAKE_SOURCE_DIR}/bin>")

add_subdirectory(./pros-sim EXCLUDE_FROM_ALL=true)
add_executable(v5_sim_engine
        src/main.cpp
        include/units.hpp
        include/Vector2D.hpp
        src/bot.cpp
        include/bot.h
        include/util.h
        Eigen/Cholesky
        Eigen/CholmodSupport
        Eigen/Core
        Eigen/Dense
        Eigen/Eigen
        Eigen/Eigenvalues
        Eigen/Geometry
        Eigen/Householder
        Eigen/IterativeLinearSolvers
        Eigen/Jacobi
        Eigen/KLUSupport
        Eigen/LU
        Eigen/MetisSupport
        Eigen/OrderingMethods
        Eigen/PardisoSupport
        Eigen/PaStiXSupport
        Eigen/QR
        Eigen/QtAlignedMalloc
        Eigen/Sparse
        Eigen/SparseCholesky
        Eigen/SparseCore
        Eigen/SparseLU
        Eigen/SparseQR
        Eigen/SPQRSupport
        Eigen/StdDeque
        Eigen/StdList
        Eigen/StdVector
        Eigen/SuperLUSupport
        Eigen/SVD
        Eigen/UmfPackSupport
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

target_include_directories(v5_sim_engine PUBLIC pros-sim/simulator/include)
target_include_directories(v5_sim_engine PUBLIC pros-sim/pros/include)
target_include_directories(v5_sim_engine PUBLIC include)
target_include_directories(v5_sim_engine PUBLIC Eigen)
add_dependencies(v5_sim_engine v5_sim_user)
target_link_libraries(v5_sim_engine PRIVATE v5_sim_user)
