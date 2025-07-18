cmake_minimum_required(VERSION 3.10)
project(Planar2LinkArm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Include dirs
include_directories(include ${EIGEN3_INCLUDE_DIR})

# Source
add_executable(planar_arm src/2_link.cpp)
