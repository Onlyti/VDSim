# Build pybind11 module for vdsim_core.
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
    list(APPEND CMAKE_PREFIX_PATH ${PYBIND11_CMAKE_DIR})
    find_package(pybind11 CONFIG)
endif()

if(pybind11_FOUND)
    pybind11_add_module(vdsim_py bindings.cpp)
    target_link_libraries(vdsim_py PRIVATE vdsim::core)
    set_target_properties(vdsim_py PROPERTIES
        OUTPUT_NAME vdsim
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
    # Wheel build (scikit-build-core): install the compiled module + the pure-
    # Python experiment layer (vdsim_lab / vdsim_comms + the map parsers) as top-
    # level modules, plus the preset configs as bundled data, so `pip install`
    # users get the builders / batch / API / comms — not just the .so.
    if(DEFINED SKBUILD)
        install(TARGETS vdsim_py LIBRARY DESTINATION ".")
        install(FILES
            ${CMAKE_SOURCE_DIR}/python/vdsim_lab.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_comms.py
            ${CMAKE_SOURCE_DIR}/examples/opendrive.py
            ${CMAKE_SOURCE_DIR}/examples/rd5_route.py
            DESTINATION ".")
        install(DIRECTORY
            ${CMAKE_SOURCE_DIR}/configs/vehicles
            ${CMAKE_SOURCE_DIR}/configs/tires
            ${CMAKE_SOURCE_DIR}/configs/roads
            DESTINATION "vdsim_configs"
            FILES_MATCHING PATTERN "*.yaml")
    endif()
    message(STATUS "Python bindings: ENABLED")
else()
    message(STATUS "Python bindings: DISABLED (pybind11 not found)")
endif()
