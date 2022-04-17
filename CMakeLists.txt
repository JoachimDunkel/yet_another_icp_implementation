cmake_minimum_required(VERSION 3.21)
project(test_cpp)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(test_cpp main.cpp PointCloud.cpp PointCloud.hpp)

target_link_libraries(test_cpp Eigen3::Eigen)
