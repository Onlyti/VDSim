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

void wheel_world_positions(const State& vehicle, const VehicleParams& vp,
                           std::array<Vec3, NUM_WHEELS>& pw) {
    const double a = vp.cg_to_front, b = vp.cg_to_rear;
    const double tf2 = 0.5 * vp.track_front, tr2 = 0.5 * vp.track_rear;
    const double hz  = -(vp.cg_height - vp.wheel_radius_nominal);
    const Vec3 body_offsets[NUM_WHEELS] = {
        Vec3(a, tf2, hz), Vec3(a, -tf2, hz),
        Vec3(-b, tr2, hz), Vec3(-b, -tr2, hz)};
    for (int i = 0; i < NUM_WHEELS; ++i) {
        // L5 free-3D: the wheel centre is a genuine inertial particle (unsprung_pos),
        // which can deviate from the body-rigid mount. Evaluate the contact at the ACTUAL
        // wheel position so penetration is exact on curved surfaces (no planar extrapolation
        // from the rigid hub -> no false contact when the wheel leaves a loop/bank). Other
        // levels leave unsprung_pos zero and use the rigid body-attached position.
        pw[i] = (vehicle.unsprung_pos[i].squaredNorm() > 1e-12)
                    ? vehicle.unsprung_pos[i]
                    : vehicle.position + vehicle.orientation * body_offsets[i];
    }
}

inline double hub_penetration(const Vec3& hub, const Vec3& n_unit,
                              const Vec3& road_pt, double wheel_r) {
    const Vec3 target = road_pt + n_unit * wheel_r;
    return std::max(0.0, (target - hub).dot(n_unit));
}

class FlatGround final : public IContactProvider {
public:
    FlatGround(double z, double mu) : z_(z), mu_(mu) {}

    void query(const State& vehicle,
               const VehicleParams& vp,
               ContactArray& out) override {
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const Vec3 n = Vec3::UnitZ();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z_);
            out[i].is_valid    = true;
            out[i].normal      = n;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n, road_pt, r);
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
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const Vec3 n = Vec3::UnitZ();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double mu = (pw[i].y() >= by_) ? mu_l_ : mu_r_;
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z_);
            out[i].is_valid    = true;
            out[i].normal      = n;
            out[i].mu_long     = mu;
            out[i].mu_lat      = mu;
            out[i].surface_id  = (pw[i].y() >= by_) ? 0 : 1;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n, road_pt, r);
        }
    }

private:
    double z_, mu_l_, mu_r_, by_;
};

// Piecewise-x friction on a flat plane: each patch [x0, x1] with linear blend
// over blend_dist_ before x0 and after x1; overlapping patches -> min mu.
class FrictionPatchGround final : public IContactProvider {
public:
    FrictionPatchGround(double z, double base_mu,
                        std::vector<std::tuple<double, double, double>> patches)
        : z_(z), base_mu_(base_mu), blend_dist_(1.0), patches_(std::move(patches)) {}

    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const Vec3 n = Vec3::UnitZ();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double mu = mu_at_x(pw[i].x());
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z_);
            out[i].is_valid    = true;
            out[i].normal      = n;
            out[i].mu_long     = mu;
            out[i].mu_lat      = mu;
            out[i].surface_id  = 0;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n, road_pt, r);
        }
    }

private:
    double mu_at_x(double x) const {
        double mu = base_mu_;
        for (const auto& [x0, x1, mu_p] : patches_) {
            double mu_here;
            if (x >= x0 && x <= x1) {
                mu_here = mu_p;
            } else if (x >= x0 - blend_dist_ && x < x0) {
                const double t = (x - (x0 - blend_dist_)) / blend_dist_;
                mu_here = base_mu_ + t * (mu_p - base_mu_);
            } else if (x > x1 && x <= x1 + blend_dist_) {
                const double t = (x - x1) / blend_dist_;
                mu_here = mu_p + t * (base_mu_ - mu_p);
            } else {
                mu_here = base_mu_;
            }
            mu = std::min(mu, mu_here);
        }
        return mu;
    }

    double z_, base_mu_, blend_dist_;
    std::vector<std::tuple<double, double, double>> patches_;
};

bool point_in_polygon(double px, double py,
                      const std::vector<std::pair<double, double>>& poly) {
    const std::size_t n = poly.size();
    if (n < 3) return false;
    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = poly[i].first,  yi = poly[i].second;
        const double xj = poly[j].first,  yj = poly[j].second;
        const bool intersect = ((yi > py) != (yj > py))
            && (px < (xj - xi) * (py - yi) / std::max(1e-12, yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

double dist_point_to_segment(double px, double py,
                             double ax, double ay, double bx, double by) {
    const double abx = bx - ax, aby = by - ay;
    const double apx = px - ax, apy = py - ay;
    const double ab2 = abx * abx + aby * aby;
    const double t = ab2 > 1e-18
        ? std::clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0) : 0.0;
    const double cx = ax + t * abx, cy = ay + t * aby;
    return std::hypot(px - cx, py - cy);
}

double point_to_polygon_boundary_dist(double px, double py,
    const std::vector<std::pair<double, double>>& poly) {
    if (poly.size() < 3) return std::numeric_limits<double>::infinity();
    if (point_in_polygon(px, py, poly)) return 0.0;
    double dmin = std::numeric_limits<double>::infinity();
    const std::size_t n = poly.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        dmin = std::min(dmin, dist_point_to_segment(
            px, py, poly[j].first, poly[j].second,
            poly[i].first, poly[i].second));
    }
    return dmin;
}

// 2-D polygon patches on a flat plane; mu blends linearly to base_mu over blend_dist.
class PolygonFrictionGround final : public IContactProvider {
public:
    PolygonFrictionGround(double z, double base_mu,
                          std::vector<PolygonMuPatch> patches,
                          double blend_dist)
        : z_(z), base_mu_(base_mu), blend_dist_(std::max(1e-6, blend_dist)),
          patches_(std::move(patches)) {}

    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const Vec3 n = Vec3::UnitZ();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double mu = mu_at_xy(pw[i].x(), pw[i].y());
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z_);
            out[i].is_valid    = true;
            out[i].normal      = n;
            out[i].mu_long     = mu;
            out[i].mu_lat      = mu;
            out[i].surface_id  = 0;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n, road_pt, r);
        }
    }

private:
    double mu_at_xy(double x, double y) const {
        double mu = base_mu_;
        for (const auto& patch : patches_) {
            const double d = point_to_polygon_boundary_dist(x, y, patch.polygon);
            double mu_p;
            if (d <= 0.0) {
                mu_p = patch.mu;
            } else if (d < blend_dist_) {
                const double t = d / blend_dist_;
                mu_p = patch.mu + t * (base_mu_ - patch.mu);
            } else {
                mu_p = base_mu_;
            }
            mu = std::min(mu, mu_p);
        }
        return mu;
    }

    double z_, base_mu_, blend_dist_;
    std::vector<PolygonMuPatch> patches_;
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
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const double z_cg = z0_ + sx_ * vehicle.position.x() + sy_ * vehicle.position.y();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double z = z0_ + sx_ * pw[i].x() + sy_ * pw[i].y();
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z);
            out[i].is_valid    = true;
            out[i].normal      = n_;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n_, road_pt, r);
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
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const Vec3 n = Vec3::UnitZ();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double dz = profile(pw[i].x());
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z_ + dz);
            out[i].is_valid    = true;
            out[i].normal      = n;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n, road_pt, r);
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
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const double e = 0.25 * std::min(dx_, dy_);
        const double z_cg = height(vehicle.position.x(), vehicle.position.y());
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double z = height(pw[i].x(), pw[i].y());
            const double dhdx = (height(pw[i].x() + e, pw[i].y())
                               - height(pw[i].x() - e, pw[i].y())) / (2 * e);
            const double dhdy = (height(pw[i].x(), pw[i].y() + e)
                               - height(pw[i].x(), pw[i].y() - e)) / (2 * e);
            const Vec3 n(-dhdx, -dhdy, 1.0);
            const Vec3 n_unit = n.normalized();
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z);
            out[i].is_valid    = true;
            out[i].normal      = n_unit;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n_unit, road_pt, r);
            out[i].road_dz     = z - z_cg;
        }
    }

private:
    std::vector<double> h_;
    int nx_, ny_;
    double x0_, y0_, dx_, dy_, mu_;
};

// Road profile synthesizer from a displacement PSD Gd(n) [m^3] over spatial
// frequency n [cycles/m]. Built as a sum of sinusoids
//   z(x) = sum_i sqrt(2 Gd(n_i) dn_i) cos(2 pi n_i x + phi_i)
// over log-spaced bands, with independent seeded left/right tracks (roll
// excitation; the wheelbase gap between front/rear samples gives pitch). Gd(n)
// is supplied as any callable -- ISO 8608 single/dual-slope analytic, or a
// measured (n, Gd) table -- so different surfaces with the same ISO class but
// different spectra (Belgian pavé, washboard, curb) are distinguishable.
class PsdProfile {
public:
    template <class GdFn>
    PsdProfile(GdFn gd, double n_min, double n_max, unsigned seed) {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> uni(0.0, 2.0 * M_PI);
        const double lo = std::log(std::max(1e-4, n_min));
        const double hi = std::log(std::max(n_min * 1.01, n_max));
        for (int i = 0; i < N_; ++i) {
            const double f0 = std::exp(lo + (hi - lo) * i / N_);
            const double f1 = std::exp(lo + (hi - lo) * (i + 1) / N_);
            const double ni = 0.5 * (f0 + f1), dni = f1 - f0;
            n_[i]   = ni;
            amp_[i] = std::sqrt(2.0 * std::max(0.0, gd(ni)) * dni);
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

inline double iso_class_gd0(int road_class) {
    // geometric-mean Gd(n0) per ISO 8608 class A..H [m^3] (each class x4)
    static const double GD[8] = {16e-6, 64e-6, 256e-6, 1024e-6,
                                  4096e-6, 16384e-6, 65536e-6, 262144e-6};
    return GD[std::clamp(road_class, 0, 7)];
}

// Analytic PSD: single slope Gd(n)=Gd0 (n/n0)^-w, or a continuous dual-slope
// with exponent w below n_break and w_high above (richer high-freq content,
// e.g. Belgian pavé / cobblestone). n_break<=0 -> single slope.
struct AnalyticGd {
    double gd0, w, n_break, w_high;
    static constexpr double n0 = 0.1;
    double operator()(double n) const {
        if (n_break > 0.0 && n > n_break) {
            const double gdb = gd0 * std::pow(n_break / n0, -w);
            return gdb * std::pow(n / n_break, -w_high);
        }
        return gd0 * std::pow(n / n0, -w);
    }
};

// Measured PSD as a (n, Gd) table; log-log interpolation, clamp at the ends.
// Lets users plug a proving-ground RLDA spectrum directly.
struct TableGd {
    std::vector<double> n, gd;   // n ascending
    double operator()(double q) const {
        if (n.empty()) return 0.0;
        if (q <= n.front()) return gd.front();
        if (q >= n.back())  return gd.back();
        const auto it = std::lower_bound(n.begin(), n.end(), q);
        const std::size_t j = static_cast<std::size_t>(it - n.begin());
        const double t = (std::log(q) - std::log(n[j - 1])) /
                         (std::log(n[j]) - std::log(n[j - 1]));
        return std::exp(std::log(gd[j - 1]) + t * (std::log(gd[j]) - std::log(gd[j - 1])));
    }
};

// Ground whose per-wheel road_dz comes from a PsdProfile (any Gd source).
class PsdGround final : public IContactProvider {
public:
    PsdGround(double z, double mu, PsdProfile prof) : z_(z), mu_(mu), prof_(std::move(prof)) {}
    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const Vec3 n = Vec3::UnitZ();
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const bool right = (i == WHEEL_FR || i == WHEEL_RR);
            const double dz = prof_.height(pw[i].x(), right);
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z_ + dz);
            out[i].is_valid    = true;
            out[i].normal      = n;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 0;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n, road_pt, r);
            out[i].road_dz     = dz;
        }
    }
private:
    double z_, mu_;
    PsdProfile prof_;
};

// T23 half-cosine ramp + lip + cliff (1D in world x).
class RampGround final : public IContactProvider {
public:
    RampGround(double x_start, double x_top, double height, double lip, double mu)
        : xs_(x_start), xt_(x_top), h_(height), lip_(lip), mu_(mu),
          xl_(x_top + lip) {}

    static double profile_z(double x, double xs, double xt, double h, double xl) {
        if (x < xs) return 0.0;
        if (x < xt) {
            const double u = (x - xs) / std::max(1e-6, xt - xs);
            return h * 0.5 * (1.0 - std::cos(M_PI * u));
        }
        if (x < xl) return h;
        return 0.0;
    }

    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        constexpr double kAirReach = 0.22;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double x = pw[i].x();
            const double z = profile_z(x, xs_, xt_, h_, xl_);
            const double eps = 0.05;
            const double zl = profile_z(x - eps, xs_, xt_, h_, xl_);
            const double zr = profile_z(x + eps, xs_, xt_, h_, xl_);
            const Vec3 n = Vec3(-(zr - zl) / (2.0 * eps), 0.0, 1.0).normalized();
            const Vec3 road_pt(x, pw[i].y(), z);
            const Vec3 target = road_pt + n * r;
            const double pen = std::max(0.0, (target - pw[i]).dot(n));
            const bool past_lip = x >= xl_;
            if (past_lip && pen <= 0.0 && pw[i].z() > z + kAirReach) {
                out[i].is_valid    = false;
                out[i].penetration = 0.0;
                continue;
            }
            out[i].is_valid    = pen > 0.0;
            out[i].normal      = n;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 1;
            out[i].position    = road_pt;
            out[i].penetration = pen;
            out[i].road_dz     = z;
        }
    }

private:
    double xs_, xt_, h_, lip_, xl_, mu_;
};

// Vertical loop in x-z: a circular wall of radius R about (xc, zc); the car drives
// the INSIDE, so the surface normal points inward (toward the centre) and the tyre
// contact patch rolls on the radius-R circle (wheel centre at R - r). Physical penalty
// contact like the flat/curved grounds — no rail/reach hack; staying on the loop is
// emergent (the contact normal supplies the centripetal force only when fast enough).
class LoopGround final : public IContactProvider {
public:
    LoopGround(double xc, double zc, double radius, double mu)
        : xc_(xc), zc_(zc), R_(std::max(2.0, radius)), mu_(mu) {}

    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        const double contact_rad = R_ - r;          // wheel-centre radius at first touch
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double dx = pw[i].x() - xc_;
            const double dz = pw[i].z() - zc_;
            const double hub_rad = std::hypot(dx, dz);
            const double inv = hub_rad > 1e-6 ? 1.0 / hub_rad : 0.0;
            const Vec3 n(-dx * inv, 0.0, -dz * inv);              // inward (toward centre)
            const double pen = hub_rad - contact_rad;            // >0 = tyre into the wall
            out[i].is_valid    = hub_rad > 0.4 * R_ && hub_rad < R_ + 0.5;
            out[i].normal      = n;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 2;
            out[i].position    = Vec3(xc_ + R_ * dx * inv, pw[i].y(), zc_ + R_ * dz * inv);
            out[i].penetration = std::max(0.0, pen);
            out[i].road_dz     = out[i].position.z();
        }
    }

    double radius() const { return R_; }
    double v_min() const { return std::sqrt(5.0 * 9.80665 * R_); }

private:
    double xc_, zc_, R_, mu_;
};

// Banked circular turn in the x-y plane: reference circle radius R about
// (xc, yc), banked inward by `bank` (outer edge higher). Height rises with the
// radial distance rho: z = z0 + (rho - R)*tan(bank); the surface normal tilts
// toward the centre and up, n ~ (-tan(bank)*e_rho, 1), so the normal force has a
// centripetal component (the physics of a velodrome / oval turn).
class CurvedGround final : public IContactProvider {
public:
    CurvedGround(double xc, double yc, double radius, double bank,
                 double z0, double mu)
        : xc_(xc), yc_(yc), R_(std::max(2.0, radius)),
          tb_(std::tan(bank)), z0_(z0), mu_(mu) {}

    void query(const State& vehicle, const VehicleParams& vp,
               ContactArray& out) override {
        std::array<Vec3, NUM_WHEELS> pw{};
        wheel_world_positions(vehicle, vp, pw);
        const double r = vp.wheel_radius_nominal;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double ex = pw[i].x() - xc_, ey = pw[i].y() - yc_;
            const double rho = std::hypot(ex, ey);
            const double inv = rho > 1e-6 ? 1.0 / rho : 0.0;
            const double rx = ex * inv, ry = ey * inv;   // outward radial unit
            const double z = z0_ + (rho - R_) * tb_;
            const Vec3 n = Vec3(-tb_ * rx, -tb_ * ry, 1.0).normalized();
            const Vec3 road_pt(pw[i].x(), pw[i].y(), z);
            out[i].is_valid    = true;
            out[i].normal      = n;
            out[i].mu_long     = mu_;
            out[i].mu_lat      = mu_;
            out[i].surface_id  = 3;
            out[i].position    = road_pt;
            out[i].penetration = hub_penetration(pw[i], n, road_pt, r);
        }
    }

private:
    double xc_, yc_, R_, tb_, z0_, mu_;
};

class FlatRoughness final : public IRoughnessProvider {
public:
    double sample_height(const Vec2& /*world_xy*/) const override { return 0.0; }
};

// 1-D ISO 8608 roughness as an IRoughnessProvider (sample_height along world x).
class Iso8608Roughness final : public IRoughnessProvider {
public:
    Iso8608Roughness(int road_class, unsigned seed)
        : prof_(AnalyticGd{iso_class_gd0(road_class), 2.0, 0.0, 2.0}, 0.011, 4.0, seed) {}
    double sample_height(const Vec2& world_xy) const override {
        return prof_.height(world_xy.x(), false);
    }
private:
    PsdProfile prof_;
};

}  // namespace

std::unique_ptr<IContactProvider> create_flat_ground(double z, double mu) {
    return std::make_unique<FlatGround>(z, mu);
}

std::unique_ptr<IContactProvider> create_split_mu_ground(
    double z, double mu_left, double mu_right, double boundary_y) {
    return std::make_unique<SplitMuGround>(z, mu_left, mu_right, boundary_y);
}

std::unique_ptr<IContactProvider> create_friction_patch_ground(
    double z, double base_mu,
    const std::vector<std::tuple<double, double, double>>& patches) {
    return std::make_unique<FrictionPatchGround>(z, base_mu, patches);
}

std::unique_ptr<IContactProvider> create_polygon_friction_ground(
    double z, double base_mu,
    const std::vector<PolygonMuPatch>& patches,
    double blend_distance) {
    return std::make_unique<PolygonFrictionGround>(
        z, base_mu, patches, blend_distance);
}

std::unique_ptr<IContactProvider> create_inclined_ground(
    double z0, double grade, double bank, double mu) {
    return std::make_unique<InclinedGround>(z0, grade, bank, mu);
}

std::unique_ptr<IContactProvider> create_ramp_ground(
    double x_start, double x_top, double height, double lip_length, double mu) {
    return std::make_unique<RampGround>(x_start, x_top, height, lip_length, mu);
}

std::unique_ptr<IContactProvider> create_loop_ground(
    double xc, double zc, double radius, double mu) {
    return std::make_unique<LoopGround>(xc, zc, radius, mu);
}

std::unique_ptr<IContactProvider> create_curved_ground(
    double xc, double yc, double radius, double bank, double z0, double mu) {
    return std::make_unique<CurvedGround>(xc, yc, radius, bank, z0, mu);
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
    return std::make_unique<PsdGround>(
        z, mu, PsdProfile(AnalyticGd{iso_class_gd0(road_class), 2.0, 0.0, 2.0},
                          0.011, 4.0, seed));
}

std::unique_ptr<IContactProvider> create_psd_ground(
    double z, double mu, double gd_n0, double waviness, double n_break,
    double waviness_high, double n_min, double n_max, unsigned seed) {
    return std::make_unique<PsdGround>(
        z, mu, PsdProfile(AnalyticGd{gd_n0, waviness, n_break, waviness_high},
                          n_min, n_max, seed));
}

std::unique_ptr<IContactProvider> create_psd_ground_table(
    double z, double mu, std::vector<double> n, std::vector<double> gd,
    double n_min, double n_max, unsigned seed) {
    return std::make_unique<PsdGround>(
        z, mu, PsdProfile(TableGd{std::move(n), std::move(gd)}, n_min, n_max, seed));
}

std::unique_ptr<IRoughnessProvider> create_flat() {
    return std::make_unique<FlatRoughness>();
}

std::unique_ptr<IRoughnessProvider> create_iso8608_psd(int grade) {
    // grade as 0=A..7=H; legacy callers may pass 1=A..5=E, clamp handles both.
    return std::make_unique<Iso8608Roughness>(std::clamp(grade, 0, 7), 1u);
}

}  // namespace vdsim
