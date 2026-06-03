// Contact and roughness providers.
// - FlatGround: 4-wheel raycast against a horizontal plane at z = z0,
//   constant friction mu. PoC default for unit / integration tests.
// - FlatRoughness: zero terrain roughness.
// - ISO 8608 PSD roughness: not yet implemented.

#include "vdsim/interfaces.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

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
        // per-wheel road height relative to the CG ground point: the tilt part of
        // this differential drives the L3 ride model's roll/pitch attitude.
        const double z_cg = z0_ + sx_ * vehicle.position.x() + sy_ * vehicle.position.y();
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
            out[i].road_dz     = z - z_cg;
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

// Heightmap terrain: a regular grid h[iy*nx+ix] at (x0+ix*dx, y0+iy*dy).
// Per-wheel bilinear height + surface normal from the local gradient (so the
// slope-gravity in the dynamics works on arbitrary terrain). A Blender mesh is
// used by baking it to a heightmap. Queries outside the grid clamp to the edge.
class HeightmapGround final : public IContactProvider {
public:
    HeightmapGround(std::vector<double> h, int nx, int ny,
                    double x0, double y0, double dx, double dy, double mu)
        : h_(std::move(h)), nx_(nx), ny_(ny), x0_(x0), y0_(y0),
          dx_(dx), dy_(dy), mu_(mu) {}

    double height(double x, double y) const {
        double fx = (x - x0_) / dx_, fy = (y - y0_) / dy_;
        int ix = std::clamp(int(std::floor(fx)), 0, nx_ - 2);
        int iy = std::clamp(int(std::floor(fy)), 0, ny_ - 2);
        const double tx = std::clamp(fx - ix, 0.0, 1.0);
        const double ty = std::clamp(fy - iy, 0.0, 1.0);
        const double h00 = h_[iy * nx_ + ix],       h10 = h_[iy * nx_ + ix + 1];
        const double h01 = h_[(iy + 1) * nx_ + ix], h11 = h_[(iy + 1) * nx_ + ix + 1];
        return (h00 * (1 - tx) + h10 * tx) * (1 - ty)
             + (h01 * (1 - tx) + h11 * tx) * ty;
    }
    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        const double a = vp.cg_to_front, b = vp.cg_to_rear;
        const double tf2 = 0.5 * vp.track_front, tr2 = 0.5 * vp.track_rear;
        const Vec3 body_offsets[NUM_WHEELS] = {
            Vec3(a, tf2, 0.0), Vec3(a, -tf2, 0.0),
            Vec3(-b, tr2, 0.0), Vec3(-b, -tr2, 0.0)};
        const double e = 0.25 * std::min(dx_, dy_);   // gradient finite-diff step
        const double z_cg = height(vehicle.position.x(), vehicle.position.y());
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 pw = vehicle.position + vehicle.orientation * body_offsets[i];
            const double z = height(pw.x(), pw.y());
            const double dhdx = (height(pw.x() + e, pw.y()) - height(pw.x() - e, pw.y())) / (2 * e);
            const double dhdy = (height(pw.x(), pw.y() + e) - height(pw.x(), pw.y() - e)) / (2 * e);
            out[i].is_valid    = true;
            out[i].normal      = Vec3(-dhdx, -dhdy, 1.0).normalized();
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = Vec3(pw.x(), pw.y(), z);
            out[i].penetration = std::max(0.0, vehicle.position.z() - z);
            // per-wheel road height relative to the CG ground point -> L3 attitude
            out[i].road_dz     = z - z_cg;
        }
    }

private:
    std::vector<double> h_;
    int nx_, ny_;
    double x0_, y0_, dx_, dy_, mu_;
};

// ISO 8608 road profile synthesizer. The standard one-sided displacement PSD is
//   Gd(n) = Gd(n0) * (n/n0)^-w,   n0 = 0.1 cycles/m, w = 2,
// with Gd(n0) the road-class roughness coefficient. A profile with that PSD is
// built as a sum of sinusoids z(x) = sum_i sqrt(2 Gd(n_i) dn_i) cos(2 pi n_i x +
// phi_i) over log-spaced spatial frequencies, phases random (seeded). Two
// independent tracks (left/right) so the road excites roll as well as bounce.
class Iso8608Profile {
public:
    Iso8608Profile(int road_class, unsigned seed) {
        // geometric-mean Gd(n0) per class A..H [m^3] (each class x4)
        static const double GD[8] = {16e-6, 64e-6, 256e-6, 1024e-6,
                                      4096e-6, 16384e-6, 65536e-6, 262144e-6};
        const double gd0 = GD[std::clamp(road_class, 0, 7)];
        const double n0 = 0.1, n_min = 0.011, n_max = 4.0;   // wavelength ~0.25..90 m
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> uni(0.0, 2.0 * M_PI);
        const double logmin = std::log(n_min), logmax = std::log(n_max);
        for (int i = 0; i < N_; ++i) {
            // log-spaced band centers + bin width for the sqrt(2 Gd dn) amplitude
            const double f0 = std::exp(logmin + (logmax - logmin) * i / N_);
            const double f1 = std::exp(logmin + (logmax - logmin) * (i + 1) / N_);
            const double ni = 0.5 * (f0 + f1), dni = f1 - f0;
            const double gd = gd0 * std::pow(ni / n0, -2.0);
            n_[i]  = ni;
            amp_[i] = std::sqrt(2.0 * gd * dni);
            phL_[i] = uni(rng);
            phR_[i] = uni(rng);
        }
    }
    double height(double x, bool right) const {
        double z = 0.0;
        for (int i = 0; i < N_; ++i)
            z += amp_[i] * std::cos(2.0 * M_PI * n_[i] * x + (right ? phR_[i] : phL_[i]));
        return z;
    }
private:
    static constexpr int N_ = 200;
    double n_[N_], amp_[N_], phL_[N_], phR_[N_];
};

class Iso8608Ground final : public IContactProvider {
public:
    Iso8608Ground(double z, double mu, int road_class, unsigned seed)
        : z_(z), mu_(mu), prof_(road_class, seed) {}
    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        const double a = vp.cg_to_front, b = vp.cg_to_rear;
        const double tf2 = 0.5 * vp.track_front, tr2 = 0.5 * vp.track_rear;
        const Vec3 body_offsets[NUM_WHEELS] = {
            Vec3(a, tf2, 0.0), Vec3(a, -tf2, 0.0),
            Vec3(-b, tr2, 0.0), Vec3(-b, -tr2, 0.0)};
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 pw = vehicle.position + vehicle.orientation * body_offsets[i];
            const bool right = (i == WHEEL_FR || i == WHEEL_RR);
            const double dz = prof_.height(pw.x(), right);   // profile along travel
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
    double z_, mu_;
    Iso8608Profile prof_;
};

class FlatRoughness final : public IRoughnessProvider {
public:
    double sample_height(const Vec2& /*world_xy*/) const override { return 0.0; }
};

// 1-D ISO 8608 roughness as an IRoughnessProvider (sample_height along world x).
class Iso8608Roughness final : public IRoughnessProvider {
public:
    Iso8608Roughness(int road_class, unsigned seed) : prof_(road_class, seed) {}
    double sample_height(const Vec2& world_xy) const override {
        return prof_.height(world_xy.x(), false);
    }
private:
    Iso8608Profile prof_;
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

std::unique_ptr<IContactProvider> create_heightmap_ground(
    std::vector<double> h, int nx, int ny,
    double x0, double y0, double dx, double dy, double mu) {
    return std::make_unique<HeightmapGround>(
        std::move(h), nx, ny, x0, y0, dx, dy, mu);
}

std::unique_ptr<IContactProvider> create_iso8608_ground(
    double z, double mu, int road_class, unsigned seed) {
    return std::make_unique<Iso8608Ground>(z, mu, road_class, seed);
}

std::unique_ptr<IRoughnessProvider> create_flat() {
    return std::make_unique<FlatRoughness>();
}

std::unique_ptr<IRoughnessProvider> create_iso8608_psd(int grade) {
    // grade as 0=A..7=H; legacy callers may pass 1=A..5=E, clamp handles both.
    return std::make_unique<Iso8608Roughness>(std::clamp(grade, 0, 7), 1u);
}

}  // namespace vdsim
