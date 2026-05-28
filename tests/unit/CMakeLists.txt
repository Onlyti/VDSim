add_executable(vdsim_unit_tests
    test_version.cpp
)

target_link_libraries(vdsim_unit_tests PRIVATE
    vdsim::core
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(vdsim_unit_tests)
