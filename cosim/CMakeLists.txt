add_executable(vdsim_realtime realtime_server.cpp world_scenario.cpp)
target_include_directories(vdsim_realtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vdsim_realtime PRIVATE vdsim::core)
if(WIN32)
    target_link_libraries(vdsim_realtime PRIVATE ws2_32)   # Winsock2
endif()
# Flat output dir for every config so the GUI finds build/bin/vdsim_realtime[.exe]
# (VS multi-config would otherwise nest it under build/bin/Release).
set_target_properties(vdsim_realtime PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY                ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${CMAKE_BINARY_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${CMAKE_BINARY_DIR}/bin)
