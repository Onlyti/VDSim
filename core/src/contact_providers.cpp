// Contact and roughness providers.
// - FlatGround: 4-wheel raycast against a horizontal plane at z = z0,
//   constant friction mu. PoC default for unit / integration tests.
// - FlatRoughness: zero terrain roughness.
// - ISO 8608 PSD roughness: not yet implemented.

#include "vdsim/interfaces.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

namespace vdsim {

namespace {

class FlatGround final : public IContactProvider {
public:
    FlatGround(double z, double mu) : z_(z), mu_(mu) {}

    void query(const State& vehicle,
               const VehicleParams& vp,
               ContactArray& out) override {
        // Wheel offsets in body frame (FL, FR, RL, RR).
        const double a   = vp.cg_to_front;
        const double b   = vp.cg_to_rear;
        const double tf2 = 0.5 * vp.track_front;
        const double tr2 = 0.5 * vp.track_rear;

        const Vec3 body_offsets[NUM_WHEELS] = {
            Vec3( a,  tf2, 0.0),   // FL
            Vec3( a, -tf2, 0.0),   // FR
            Vec3(-b,  tr2, 0.0),   // RL
            Vec3(-b, -tr2, 0.0),   // RR
        };

        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 pos_world = vehicle.position +
                                   vehicle.orientation * body_offsets[i];
            out[i].is_valid    = true;
            out[i].normal      = Vec3::UnitZ();
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = Vec3(pos_world.x(), pos_world.y(), z_);
            out[i].penetration = std::max(0.0, vehicle.position.z() - z_);
        }
    }

private:
    double z_;
    double mu_;
};

class FlatRoughness final : public IRoughnessProvider {
public:
    double sample_height(const Vec2& /*world_xy*/) const override { return 0.0; }
};

}  // namespace

std::unique_ptr<IContactProvider> create_flat_ground(double z, double mu) {
    return std::make_unique<FlatGround>(z, mu);
}

std::unique_ptr<IRoughnessProvider> create_flat() {
    return std::make_unique<FlatRoughness>();
}

std::unique_ptr<IRoughnessProvider> create_iso8608_psd(int /*grade*/) {
    throw std::runtime_error(
        "vdsim::create_iso8608_psd: ISO 8608 PSD roughness not yet implemented");
}

}  // namespace vdsim
