include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(runICPTests
        affine_transform_test.cpp)

target_link_libraries(runICPTests gtest gtest_main)
target_link_libraries(runICPTests fast_icp_core)