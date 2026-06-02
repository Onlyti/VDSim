add_executable(vdsim_udp_server udp_server.cpp)
target_include_directories(vdsim_udp_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vdsim_udp_server PRIVATE vdsim::core)
set_target_properties(vdsim_udp_server PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
