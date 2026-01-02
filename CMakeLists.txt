cmake_minimum_required(VERSION 3.20)
project(MedicalMotionProfiler)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
add_executable(profiler src/main.cpp src/MotionProfiler.cpp)
target_include_directories(profiler PRIVATE include)
target_link_libraries(profiler Eigen3::Eigen)
enable_testing()
