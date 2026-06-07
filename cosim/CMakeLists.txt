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

find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_FOUND)
    add_test(NAME cosim_multi_vehicle
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/cosim/test_multi_vehicle.py)
    set_tests_properties(cosim_multi_vehicle PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 30)
    add_test(NAME l3_scenario_susp_paths
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_l3_scenario_paths.py)
    set_tests_properties(l3_scenario_susp_paths PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_test(NAME runconfig_roundtrip
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_runconfig_roundtrip.py)
    set_tests_properties(runconfig_roundtrip PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/cosim:${CMAKE_SOURCE_DIR}/gui")
    add_test(NAME catalog_resolver
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_catalog_resolver.py)
    set_tests_properties(catalog_resolver PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
endif()
