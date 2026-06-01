#include "raycast_contact_provider.hpp"

#include <algorithm>
#include <cmath>

namespace vdsim_carla {

namespace {

vdsim::Vec3 wheel_world_xy(const vdsim::State& s,
                            const vdsim::VehicleParams& vp,
                            int wheel_idx) {
    const double a    = vp.cg_to_front;
    const double b    = vp.cg_to_rear;
    const double tw_f = vp.track_front * 0.5;
    const double tw_r = vp.track_rear  * 0.5;
    double rx = 0.0, ry = 0.0;
    switch (wheel_idx) {
        case vdsim::WHEEL_FL: rx = +a; ry = +tw_f; break;
        case vdsim::WHEEL_FR: rx = +a; ry = -tw_f; break;
        case vdsim::WHEEL_RL: rx = -b; ry = +tw_r; break;
        case vdsim::WHEEL_RR: rx = -b; ry = -tw_r; break;
        default: break;
    }
    const double yaw = vdsim::yaw_from_quat(s.orientation);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    return {s.position.x() + cy * rx - sy * ry,
            s.position.y() + sy * rx + cy * ry,
            s.position.z() + 0.5};   // ray starts slightly above wheel center
}

}  // namespace

RaycastContactProvider::RaycastContactProvider(RaycastFn raycast,
                                                const SurfaceMaterial* materials,
                                                int n_materials,
                                                double default_mu)
    : raycast_(std::move(raycast)), default_mu_(default_mu) {
    materials_.reserve(n_materials);
    for (int i = 0; i < n_materials; ++i) materials_.push_back(materials[i]);
}

void RaycastContactProvider::query(const vdsim::State& vehicle,
                                    const vdsim::VehicleParams& vparams,
                                    vdsim::ContactArray& out) {
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        const auto start = wheel_world_xy(vehicle, vparams, i);
        double out_z = 0.0;
        vdsim::Vec3 normal = vdsim::Vec3::UnitZ();
        int surface_id = -1;
        const bool hit = raycast_ ? raycast_(start, 5.0, out_z, normal, surface_id)
                                     : false;
        if (hit) {
            out[i].is_valid    = true;
            out[i].normal      = normal;
            out[i].position    = vdsim::Vec3(start.x(), start.y(), out_z);
            out[i].surface_id  = surface_id;
            auto it = std::find_if(materials_.begin(), materials_.end(),
                [&](const SurfaceMaterial& m){ return m.id == surface_id; });
            if (it != materials_.end()) {
                out[i].mu_long = it->mu_long;
                out[i].mu_lat  = it->mu_lat;
            } else {
                out[i].mu_long = default_mu_;
                out[i].mu_lat  = default_mu_;
            }
            out[i].penetration = std::max(0.0, vparams.wheel_radius_nominal -
                                                (start.z() - out_z));
        } else {
            out[i].is_valid = false;
        }
    }
}

std::unique_ptr<vdsim::IContactProvider> create_raycast_provider(
    RaycastFn raycast,
    const RaycastContactProvider::SurfaceMaterial* materials,
    int n_materials,
    double default_mu) {
    return std::make_unique<RaycastContactProvider>(std::move(raycast),
                                                     materials, n_materials,
                                                     default_mu);
}

}  // namespace vdsim_carla
