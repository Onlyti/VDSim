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

// Split-mu plane at z: per-wheel friction picked by world-y relative to a
// boundary (y >= boundary -> mu_left, else mu_right; ISO 8855 +y = left).
// mu_left == mu_right reduces to uniform flat ground.
class SplitMuGround final : public IContactProvider {
public:
    SplitMuGround(double z, double mu_left, double mu_right, double boundary_y)
        : z_(z), mu_l_(mu_left), mu_r_(mu_right), by_(boundary_y) {}

    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        const double a = vp.cg_to_front, b = vp.cg_to_rear;
        const double tf2 = 0.5 * vp.track_front, tr2 = 0.5 * vp.track_rear;
        const Vec3 body_offsets[NUM_WHEELS] = {
            Vec3(a, tf2, 0.0), Vec3(a, -tf2, 0.0),
            Vec3(-b, tr2, 0.0), Vec3(-b, -tr2, 0.0)};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 pw = vehicle.position + vehicle.orientation * body_offsets[i];
            const double mu = (pw.y() >= by_) ? mu_l_ : mu_r_;
            out[i].is_valid    = true;
            out[i].normal      = Vec3::UnitZ();
            out[i].mu_long     = mu;
            out[i].mu_lat      = mu;
            out[i].surface_id  = (pw.y() >= by_) ? 0 : 1;
            out[i].position    = Vec3(pw.x(), pw.y(), z_);
            out[i].penetration = std::max(0.0, vehicle.position.z() - z_);
        }
    }

private:
    double z_, mu_l_, mu_r_, by_;
};

// Inclined plane: height h = z0 + tan(grade)*x + tan(bank)*y, so the surface
// normal is (-tan grade, -tan bank, 1) normalized. grade>0 rises toward +x
// (uphill ahead -> decel), bank>0 rises toward +y (left). grade=bank=0 -> flat.
class InclinedGround final : public IContactProvider {
public:
    InclinedGround(double z0, double grade, double bank, double mu)
        : z0_(z0), sx_(std::tan(grade)), sy_(std::tan(bank)), mu_(mu) {
        n_ = Vec3(-sx_, -sy_, 1.0).normalized();
    }
    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        const double a = vp.cg_to_front, b = vp.cg_to_rear;
        const double tf2 = 0.5 * vp.track_front, tr2 = 0.5 * vp.track_rear;
        const Vec3 body_offsets[NUM_WHEELS] = {
            Vec3(a, tf2, 0.0), Vec3(a, -tf2, 0.0),
            Vec3(-b, tr2, 0.0), Vec3(-b, -tr2, 0.0)};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 pw = vehicle.position + vehicle.orientation * body_offsets[i];
            const double z = z0_ + sx_ * pw.x() + sy_ * pw.y();
            out[i].is_valid    = true;
            out[i].normal      = n_;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = Vec3(pw.x(), pw.y(), z);
            out[i].penetration = std::max(0.0, vehicle.position.z() - z);
        }
    }

private:
    double z0_, sx_, sy_, mu_;
    Vec3 n_;
};

// Rough flat plane: a deterministic two-tone longitudinal road profile feeds
// ContactPoint.road_dz (and the contact height) to excite the L3 unsprung ride.
// amp=0 -> smooth. Profile sampled at the wheel's world-x.
class RoughGround final : public IContactProvider {
public:
    RoughGround(double z, double mu, double amp, double wavelength)
        : z_(z), mu_(mu), amp_(amp),
          k1_(2.0 * M_PI / std::max(0.1, wavelength)),
          k2_(2.0 * M_PI / std::max(0.1, 0.37 * wavelength)) {}

    double profile(double x) const {
        return amp_ * (std::sin(k1_ * x) + 0.5 * std::sin(k2_ * x + 1.3));
    }
    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        const double a = vp.cg_to_front, b = vp.cg_to_rear;
        const double tf2 = 0.5 * vp.track_front, tr2 = 0.5 * vp.track_rear;
        const Vec3 body_offsets[NUM_WHEELS] = {
            Vec3(a, tf2, 0.0), Vec3(a, -tf2, 0.0),
            Vec3(-b, tr2, 0.0), Vec3(-b, -tr2, 0.0)};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 pw = vehicle.position + vehicle.orientation * body_offsets[i];
            const double dz = profile(pw.x());
            out[i].is_valid    = true;
            out[i].normal      = Vec3::UnitZ();
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = Vec3(pw.x(), pw.y(), z_ + dz);
            out[i].penetration = std::max(0.0, vehicle.position.z() - z_);
            out[i].road_dz     = dz;
        }
    }

private:
    double z_, mu_, amp_, k1_, k2_;
};

class FlatRoughness final : public IRoughnessProvider {
public:
    double sample_height(const Vec2& /*world_xy*/) const override { return 0.0; }
};

}  // namespace

std::unique_ptr<IContactProvider> create_flat_ground(double z, double mu) {
    return std::make_unique<FlatGround>(z, mu);
}

std::unique_ptr<IContactProvider> create_split_mu_ground(
    double z, double mu_left, double mu_right, double boundary_y) {
    return std::make_unique<SplitMuGround>(z, mu_left, mu_right, boundary_y);
}

std::unique_ptr<IContactProvider> create_inclined_ground(
    double z0, double grade, double bank, double mu) {
    return std::make_unique<InclinedGround>(z0, grade, bank, mu);
}

std::unique_ptr<IContactProvider> create_rough_ground(
    double z, double mu, double amp, double wavelength) {
    return std::make_unique<RoughGround>(z, mu, amp, wavelength);
}

std::unique_ptr<IRoughnessProvider> create_flat() {
    return std::make_unique<FlatRoughness>();
}

std::unique_ptr<IRoughnessProvider> create_iso8608_psd(int /*grade*/) {
    throw std::runtime_error(
        "vdsim::create_iso8608_psd: ISO 8608 PSD roughness not yet implemented");
}

}  // namespace vdsim
