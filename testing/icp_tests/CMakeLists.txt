include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})


add_executable(runICPTests
        affine_transform_test.cpp util_tests.cpp icp_integration_tests.cpp)

target_link_libraries(runICPTests gtest gtest_main)
target_link_libraries(runICPTests fast_icp_core)