add_library(vdsim_core
    src/version.cpp
    src/coordinate.cpp
)

target_include_directories(vdsim_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(vdsim_core PUBLIC
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    spdlog::spdlog
)

target_compile_features(vdsim_core PUBLIC cxx_std_17)

set_target_properties(vdsim_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

add_library(vdsim::core ALIAS vdsim_core)
