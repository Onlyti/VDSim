add_executable(vdsim_bicycle_run bicycle_run.cpp)
target_link_libraries(vdsim_bicycle_run PRIVATE vdsim::core)
set_target_properties(vdsim_bicycle_run PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_scenario_run scenario_run.cpp)
target_link_libraries(vdsim_scenario_run PRIVATE vdsim::core)
set_target_properties(vdsim_scenario_run PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_l1_vs_l2 l1_vs_l2_run.cpp)
target_link_libraries(vdsim_l1_vs_l2 PRIVATE vdsim::core)
set_target_properties(vdsim_l1_vs_l2 PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_ax_track_demo ax_track_demo.cpp)
target_link_libraries(vdsim_ax_track_demo PRIVATE vdsim::core)
set_target_properties(vdsim_ax_track_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_split_mu_demo split_mu_demo.cpp)
target_link_libraries(vdsim_split_mu_demo PRIVATE vdsim::core)
set_target_properties(vdsim_split_mu_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_l3_demo l3_suspension_demo.cpp)
target_link_libraries(vdsim_l3_demo PRIVATE vdsim::core)
set_target_properties(vdsim_l3_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_driver_demo driver_demo.cpp)
target_link_libraries(vdsim_driver_demo PRIVATE vdsim::core)
set_target_properties(vdsim_driver_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_path_tracking path_tracking_demo.cpp)
target_link_libraries(vdsim_path_tracking PRIVATE vdsim::core)
set_target_properties(vdsim_path_tracking PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_mf_tire_demo mf_tire_demo.cpp)
target_link_libraries(vdsim_mf_tire_demo PRIVATE vdsim::core)
set_target_properties(vdsim_mf_tire_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_headless_run headless_run.cpp)
target_link_libraries(vdsim_headless_run PRIVATE vdsim::core)
set_target_properties(vdsim_headless_run PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(vdsim_module_check module_check.cpp)
target_link_libraries(vdsim_module_check PRIVATE vdsim::core)
set_target_properties(vdsim_module_check PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Sample user subsystem-module built as a runtime-loadable plugin (.so) — used by the
# module-plugin tests and as a worked example of the VDSIM_REGISTER_*_MODULE ABI.
add_library(custom_brake_module MODULE modules/custom_brake.cpp)
target_link_libraries(custom_brake_module PRIVATE vdsim::core)
set_target_properties(custom_brake_module PROPERTIES
    OUTPUT_NAME custom_brake PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test_modules)
