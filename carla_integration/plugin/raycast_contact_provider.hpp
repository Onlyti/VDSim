#pragma once

#include <functional>
#include <memory>

#include "vdsim/contact.hpp"
#include "vdsim/interfaces.hpp"

namespace vdsim_carla {

// Callback signature: raycast from world_position downward, returns true if a
// surface was hit; on hit, populates out_z, out_normal, out_surface_id.
using RaycastFn = std::function<bool(const vdsim::Vec3& world_xy_z_start,
                                      double max_depth,
                                      double& out_z,
                                      vdsim::Vec3& out_normal,
                                      int& out_surface_id)>;

// IContactProvider backed by a user-supplied raycast.  Surface id -> (mu_long,
// mu_lat) is looked up via a tiny lookup table you fill at construction.
class RaycastContactProvider : public vdsim::IContactProvider {
public:
    struct SurfaceMaterial {
        int    id;
        double mu_long;
        double mu_lat;
    };

    RaycastContactProvider(RaycastFn raycast,
                           const SurfaceMaterial* materials,
                           int n_materials,
                           double default_mu = 1.0);

    void query(const vdsim::State& vehicle,
               const vdsim::VehicleParams& vparams,
               vdsim::ContactArray& out) override;

private:
    RaycastFn raycast_;
    std::vector<SurfaceMaterial> materials_;
    double default_mu_;
};

std::unique_ptr<vdsim::IContactProvider> create_raycast_provider(
    RaycastFn raycast,
    const RaycastContactProvider::SurfaceMaterial* materials,
    int n_materials,
    double default_mu = 1.0);

}  // namespace vdsim_carla
