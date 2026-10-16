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
    # vdsim.__git_sha__ / __git_dirty__: refreshed on every build, so the
    # module carries the HEAD it was compiled from; vdsim.__version__ is the
    # wheel version under scikit-build and "source" otherwise.
    set(VDSIM_GIT_INFO_H ${CMAKE_CURRENT_BINARY_DIR}/generated/vdsim_git_info.h)
    add_custom_target(vdsim_git_info
        COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_SOURCE_DIR} -DOUT=${VDSIM_GIT_INFO_H}
                -P ${CMAKE_SOURCE_DIR}/cmake/vdsim_git_info.cmake
        BYPRODUCTS ${VDSIM_GIT_INFO_H}
        VERBATIM)
    add_dependencies(vdsim_py vdsim_git_info)
    target_include_directories(vdsim_py PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
    if(DEFINED SKBUILD_PROJECT_VERSION)
        set(VDSIM_PY_VERSION "${SKBUILD_PROJECT_VERSION}")
    else()
        set(VDSIM_PY_VERSION "source")
    endif()
    target_compile_definitions(vdsim_py PRIVATE VDSIM_PY_VERSION="${VDSIM_PY_VERSION}")
    # Wheel build (scikit-build-core): install the compiled module + the pure-
    # Python experiment layer (vdsim_lab + the comms protocol + map parsers) as
    # top-level modules, plus the preset configs as bundled data, so `pip
    # install` users get the API / batch / comms client — not just the .so.
    if(DEFINED SKBUILD)
        install(TARGETS vdsim_py LIBRARY DESTINATION ".")
        install(FILES
            ${CMAKE_SOURCE_DIR}/python/vdsim_guard.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_lab.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_plant.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_closedloop.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_trace.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_render.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_render3d.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_preset.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_campaign.py
            ${CMAKE_SOURCE_DIR}/python/vdsim_livemonitor.py
            ${CMAKE_SOURCE_DIR}/cosim/protocol.py
            ${CMAKE_SOURCE_DIR}/examples/opendrive.py
            ${CMAKE_SOURCE_DIR}/examples/rd5_route.py
            ${CMAKE_SOURCE_DIR}/examples/quickstart.py
            DESTINATION ".")
        install(DIRECTORY
            ${CMAKE_SOURCE_DIR}/python/catalog
            ${CMAKE_SOURCE_DIR}/python/vdsim_rl
            DESTINATION "."
            FILES_MATCHING PATTERN "*.py")
        # Live monitor page + the vendored three.js it imports (no CDN).
        install(DIRECTORY ${CMAKE_SOURCE_DIR}/python/vdsim_livemonitor_web
            DESTINATION "."
            FILES_MATCHING PATTERN "*.py" PATTERN "*.html" PATTERN "*.js")
        install(FILES ${CMAKE_SOURCE_DIR}/gui/vendor/three/three.module.js
            DESTINATION "vdsim_livemonitor_web/three")
        install(FILES ${CMAKE_SOURCE_DIR}/gui/vendor/three/addons/controls/OrbitControls.js
            DESTINATION "vdsim_livemonitor_web/three/addons/controls")
        install(DIRECTORY
            ${CMAKE_SOURCE_DIR}/configs/parts
            ${CMAKE_SOURCE_DIR}/configs/blueprints
            ${CMAKE_SOURCE_DIR}/configs/catalog
            ${CMAKE_SOURCE_DIR}/configs/roads
            ${CMAKE_SOURCE_DIR}/configs/rl
            ${CMAKE_SOURCE_DIR}/configs/vehicles
            DESTINATION "vdsim_configs"
            FILES_MATCHING
            PATTERN "*.yaml"
            PATTERN "ioniq5_pac2002.tir")
    endif()
    message(STATUS "Python bindings: ENABLED")
else()
    message(STATUS "Python bindings: DISABLED (pybind11 not found)")
endif()
