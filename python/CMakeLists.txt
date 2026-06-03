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
    # Wheel build (scikit-build-core): install the module at the wheel root so
    # `import vdsim` works after `pip install`.
    if(DEFINED SKBUILD)
        install(TARGETS vdsim_py LIBRARY DESTINATION ".")
    endif()
    message(STATUS "Python bindings: ENABLED")
else()
    message(STATUS "Python bindings: DISABLED (pybind11 not found)")
endif()
