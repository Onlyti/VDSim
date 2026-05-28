add_executable(vdsim_integration_tests
    test_bicycle_steady_state.cpp
)

target_link_libraries(vdsim_integration_tests PRIVATE
    vdsim::core
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(vdsim_integration_tests)
