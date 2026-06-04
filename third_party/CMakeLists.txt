include(FetchContent)

# Make all FetchContent targets PIC so they can link into shared objects.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# CMake >= 3.31 errors on subprojects whose cmake_minimum_required(<3.5)
# (yaml-cpp 0.8.0 = 3.4, Eigen 3.4 = 2.8). Clamp the effective minimum so the
# pinned third-party releases keep configuring. Harmless on older CMake (the
# variable is simply unused). Scoped to this directory + the fetched subdirs.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# Eigen3 (MPL2, headers only)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
set(EIGEN_BUILD_DOC     OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING       OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# yaml-cpp (MIT)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
    GIT_SHALLOW    TRUE
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

# spdlog (MIT)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.13.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(spdlog)

# GoogleTest (BSD-3) — test only
if(VDSIM_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
        GIT_SHALLOW    TRUE
    )
    set(BUILD_GMOCK   OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

message(STATUS "Third-party dependencies configured")
