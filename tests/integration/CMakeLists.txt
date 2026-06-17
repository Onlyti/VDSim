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
    test_l5_strut_validation.cpp
    test_terrain_l5.cpp
    test_belt_transient.cpp
    test_belt_validation.cpp
    test_iso_baseline.cpp
    test_effective_rolling_radius.cpp
    test_camber_migration.cpp
    test_tire_inversion.cpp
    test_tire_frame.cpp
    test_drivetrain_v2.cpp
    test_user_modules.cpp
    test_module_plugin.cpp
    test_multi_vehicle.cpp
    test_vla_plant.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../cosim/scene_loader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../cosim/world_scenario.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../parity/test_chrono_pac02_parity.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../parity/test_chrono_kc_parity.cpp
)

target_include_directories(vdsim_integration_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../cosim)

target_compile_definitions(vdsim_integration_tests PRIVATE
    VDSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    VDSIM_TEST_MODULES_DIR="${CMAKE_BINARY_DIR}/test_modules")

# The module-plugin test dlopens the sample plugin .so built by the examples CMake.
add_dependencies(vdsim_integration_tests custom_brake_module)

target_link_libraries(vdsim_integration_tests PRIVATE
    vdsim::core
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(vdsim_integration_tests)
