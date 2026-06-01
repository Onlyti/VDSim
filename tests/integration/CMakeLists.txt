add_executable(vdsim_integration_tests
    test_bicycle_steady_state.cpp
    test_step_steer_sweep.cpp
    test_accel_brake.cpp
    test_weight_transfer.cpp
    test_seven_dof.cpp
    test_fourteen_dof.cpp
    test_control_dispatch.cpp
)

target_link_libraries(vdsim_integration_tests PRIVATE
    vdsim::core
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(vdsim_integration_tests)
