add_library(vdsim_core
    src/version.cpp
    src/coordinate.cpp
    src/pacejka_mf96.cpp
    src/magic_formula_tire.cpp
    src/linear_tire.cpp
    src/bicycle_dynamics.cpp
    src/kinematic_dynamics.cpp
    src/seven_dof_dynamics.cpp
    src/fourteen_dof_dynamics.cpp
    src/free_3d_dynamics.cpp
    src/contact_providers.cpp
    src/params.cpp
    src/scenario.cpp
    src/control_converter.cpp
    src/actuator_model.cpp
    src/sensor_model.cpp
    src/sim_session.cpp
    src/default_subsystems.cpp
    src/module_plugin_loader.cpp
    src/suspension_lookup.cpp
    src/dw_native_kinematics.cpp
    src/ta_native_kinematics.cpp
    src/mp_native_kinematics.cpp
    src/fivelink_native_kinematics.cpp
    src/multibody_kinematics.cpp
    src/multibody_topology.cpp
    src/multibody_compliance.cpp
    src/multibody_dynamics.cpp
    src/multibody_hard_dae.cpp
    src/multibody_kc_sweep.cpp
    src/multibody_kc_xcheck.cpp
    src/multibody_featherstone.cpp
)

target_include_directories(vdsim_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

find_package(Threads REQUIRED)

target_link_libraries(vdsim_core PUBLIC
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    spdlog::spdlog
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

target_compile_features(vdsim_core PUBLIC cxx_std_17)

set_target_properties(vdsim_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

add_library(vdsim::core ALIAS vdsim_core)
