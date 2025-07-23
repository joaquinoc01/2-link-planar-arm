cmake_minimum_required(VERSION 3.10)
project(Planar2LinkArm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Add include path
include_directories(include)

# Create a library with your classes
add_library(arm_lib
    src/planar_2link_kinematics.cpp
    src/joint_trayectory.cpp
    src/collision_checker.cpp
)

# Link Eigen to the library
target_link_libraries(arm_lib Eigen3::Eigen)

# Create the demo executable
add_executable(planar_arm_demo src/main.cpp)

# Link the library and Eigen to the demo
target_link_libraries(planar_arm_demo arm_lib Eigen3::Eigen)
