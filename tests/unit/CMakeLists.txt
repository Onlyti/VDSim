add_executable(vdsim_unit_tests
    test_version.cpp
    test_headers_compile.cpp
    test_coordinate.cpp
    test_tire_models.cpp
    test_magic_formula_tire.cpp
    test_belt_tire.cpp
    test_powertrain.cpp
    test_params_yaml.cpp
    test_scenario_yaml.cpp
    test_control_converter.cpp
    test_control_l6_l7.cpp
    test_carla_plugin.cpp
    test_multibody_header.cpp
    test_multibody_kinematics.cpp
    test_multibody_featherstone.cpp
    test_suspension_lookup.cpp
    test_actuator.cpp
    test_sim_session.cpp
    test_kinematic.cpp
    test_road_roughness.cpp
    test_lugre_tire_yaml.cpp
    test_comms_templates.cpp
    test_warn_throttle.cpp
)

target_link_libraries(vdsim_unit_tests PRIVATE vdsim_carla_plugin)

# cosim/ is header-only for the wire protocol + TX templates (comms_templates.hpp).
target_include_directories(vdsim_unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/cosim)

target_compile_definitions(vdsim_unit_tests PRIVATE
    VDSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

target_link_libraries(vdsim_unit_tests PRIVATE
    vdsim::core
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(vdsim_unit_tests)
