cmake_minimum_required(VERSION 3.21)
project(fast_icp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wno-dev -Wno-deprecated-declarations -Wno-unused-function -Wno-comment -Wno-deprecated -Wall -Werror)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.9 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_core
        Icp.cpp
        Icp.hpp
        util.cpp
        util.h
        resources.h
        PointCloud.cpp
        PointCloud.hpp
        point_cloud_util.cpp
        point_cloud_util.hpp
        CorrespondencePicker.cpp
        CorrespondencePicker.h
        nanoflann.hpp)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_14)

target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen ${PCL_LIBRARIES})

include_directories(testing)

add_subdirectory(testing)

add_executable(${PROJECT_NAME} main.cpp )
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_core
)
