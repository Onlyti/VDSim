add_executable(vdsim_integration_tests
    test_bicycle_steady_state.cpp
    test_step_steer_sweep.cpp
    test_accel_brake.cpp
    test_weight_transfer.cpp
    test_seven_dof.cpp
    test_fourteen_dof.cpp
    test_control_dispatch.cpp
    test_lugre_tire.cpp
    test_stunt.cpp
    test_l5_driving.cpp
    test_terrain_l5.cpp
    test_belt_transient.cpp
    test_belt_validation.cpp
    test_iso_baseline.cpp
    test_tire_frame.cpp
    test_drivetrain_v2.cpp
    test_user_modules.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../parity/test_chrono_pac02_parity.cpp
)

target_compile_definitions(vdsim_integration_tests PRIVATE
    VDSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

target_link_libraries(vdsim_integration_tests PRIVATE
    vdsim::core
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(vdsim_integration_tests)
