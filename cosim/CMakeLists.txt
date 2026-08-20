add_executable(vdsim_realtime realtime_server.cpp world_scenario.cpp scene_loader.cpp)
target_include_directories(vdsim_realtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vdsim_realtime PRIVATE vdsim::core)
target_compile_definitions(vdsim_realtime PRIVATE
    VDSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
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
    target_compile_definitions(vdsim_realtime PRIVATE
        VDSIM_PYTHON="${Python3_EXECUTABLE}")
    add_test(NAME cosim_multi_vehicle
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/cosim/test_multi_vehicle.py)
    set_tests_properties(cosim_multi_vehicle PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 30
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/python")
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
    add_test(NAME scene_materialize
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_scene_materialize.py)
    set_tests_properties(scene_materialize PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME catalog_api
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_catalog_api.py)
    set_tests_properties(catalog_api PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python:${CMAKE_SOURCE_DIR}/cosim:${CMAKE_SOURCE_DIR}/gui")
    add_test(NAME import_part_pack
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_import_part_pack.py)
    set_tests_properties(import_part_pack PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME level_names
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_level_names.py)
    set_tests_properties(level_names PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME module_workshop
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_module_workshop.py)
    set_tests_properties(module_workshop PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME assembly_api
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_assembly_api.py)
    set_tests_properties(assembly_api PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME part_store
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_part_store.py)
    set_tests_properties(part_store PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME blueprint_roundtrip
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_blueprint_roundtrip.py)
    set_tests_properties(blueprint_roundtrip PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME multi_vehicle_compare
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_compare.py)
    set_tests_properties(multi_vehicle_compare PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME experiment_api
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_experiment_api.py)
    set_tests_properties(experiment_api PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME vla_plant
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_vla_plant.py)
    set_tests_properties(vla_plant PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME gui_v3_api_smoke
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_gui_v3_api_smoke.py)
    set_tests_properties(gui_v3_api_smoke PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python:${CMAKE_SOURCE_DIR}/cosim:${CMAKE_SOURCE_DIR}/gui")
    add_test(NAME gui_v3_e2e
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_gui_v3_e2e.py)
    set_tests_properties(gui_v3_e2e PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 180
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python:${CMAKE_SOURCE_DIR}/cosim:${CMAKE_SOURCE_DIR}/gui")
    add_test(NAME vla_plant_demo
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/examples/vla_plant_demo.py)
    set_tests_properties(vla_plant_demo PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME control_ladder
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/examples/control_ladder_demo.py)
    set_tests_properties(control_ladder PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/build/python:${CMAKE_SOURCE_DIR}/python")
    add_test(NAME trace_roundtrip
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_trace_roundtrip.py)
    set_tests_properties(trace_roundtrip PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/python")
    # The trace tests deliberately do NOT put build/python on the path: reading
    # and rendering a trace must work where the compiled core cannot be built.
    add_test(NAME trace_render
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_trace_render.py)
    set_tests_properties(trace_render PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 180
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/python")
    add_test(NAME trace_render_multi
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_trace_multi_render.py)
    set_tests_properties(trace_render_multi PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 300
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/python")
    add_test(NAME trace_horizons
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_trace_horizons.py)
    set_tests_properties(trace_horizons PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 300
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/python")
    add_test(NAME cmrosif_compat
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tests/scripts/test_cmrosif_compat.py)
    set_tests_properties(cmrosif_compat PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_SOURCE_DIR}/cosim")
endif()
