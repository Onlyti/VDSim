add_executable(vdsim_realtime realtime_server.cpp)
target_include_directories(vdsim_realtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vdsim_realtime PRIVATE vdsim::core)
if(WIN32)
    target_link_libraries(vdsim_realtime PRIVATE ws2_32)   # Winsock2
endif()
set_target_properties(vdsim_realtime PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
