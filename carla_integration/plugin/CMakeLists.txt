add_library(vdsim_carla_plugin STATIC
    raycast_contact_provider.cpp
)

target_include_directories(vdsim_carla_plugin PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(vdsim_carla_plugin PUBLIC vdsim::core)

target_compile_features(vdsim_carla_plugin PUBLIC cxx_std_17)
