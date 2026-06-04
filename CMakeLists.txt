cmake_minimum_required(VERSION 3.16)
project(VDSim
    VERSION 0.1.0
    DESCRIPTION "Vehicle Dynamics Simulation Platform"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(VDSIM_BUILD_TESTS         "Build unit/integration tests" ON)
option(VDSIM_BUILD_EXAMPLES      "Build CLI examples"           ON)
option(VDSIM_BUILD_PYTHON        "Build Python bindings"        OFF)
option(VDSIM_BUILD_CARLA_PLUGIN  "Build CARLA integration"      ON)
option(VDSIM_BUILD_COSIM         "Build UDP co-simulation server" ON)
option(VDSIM_WITH_ERGACCESS      "Build with CarMaker ERGAccess" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if(MSVC)
    # Read sources as UTF-8 regardless of the system code page (e.g. cp949 on a
    # Korean Windows), so UTF-8 comments/strings aren't misread — also silences
    # the spdlog C4819 warnings.
    add_compile_options(/utf-8)
endif()

# M_PI et al. are POSIX/GNU extensions: glibc/libc++ expose them by default but
# MSVC only with _USE_MATH_DEFINES before <cmath>. Define it everywhere (no-op
# on GCC/Clang) so the core's trig constants compile on Windows.
add_compile_definitions(_USE_MATH_DEFINES)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(third_party)
add_subdirectory(core)

if(VDSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(VDSIM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(VDSIM_BUILD_PYTHON)
    add_subdirectory(python)
endif()

if(VDSIM_BUILD_CARLA_PLUGIN)
    add_subdirectory(carla_integration)
endif()

if(VDSIM_BUILD_COSIM)
    add_subdirectory(cosim)
endif()

message(STATUS "VDSim version       : ${PROJECT_VERSION}")
message(STATUS "Build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler        : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build tests         : ${VDSIM_BUILD_TESTS}")
message(STATUS "Build examples      : ${VDSIM_BUILD_EXAMPLES}")
message(STATUS "Build Python        : ${VDSIM_BUILD_PYTHON}")
message(STATUS "Build CARLA plugin  : ${VDSIM_BUILD_CARLA_PLUGIN}")
message(STATUS "ERGAccess           : ${VDSIM_WITH_ERGACCESS}")
