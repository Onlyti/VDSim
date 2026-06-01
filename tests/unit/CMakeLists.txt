add_executable(vdsim_unit_tests
    test_version.cpp
    test_headers_compile.cpp
    test_coordinate.cpp
    test_tire_models.cpp
    test_params_yaml.cpp
    test_scenario_yaml.cpp
    test_control_converter.cpp
    test_control_l6_l7.cpp
    test_carla_plugin.cpp
    test_multibody_header.cpp
    test_suspension_lookup.cpp
)

target_link_libraries(vdsim_unit_tests PRIVATE vdsim_carla_plugin)

target_compile_definitions(vdsim_unit_tests PRIVATE
    VDSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

target_link_libraries(vdsim_unit_tests PRIVATE
    vdsim::core
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(vdsim_unit_tests)
