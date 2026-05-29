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
