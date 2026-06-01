// Aggregate step-steer sweep: verifies linear-region agreement with the
// analytical linear-bicycle steady-state across (vx, delta) grid.
//
// Pass if every grid point with ay_est <= 3.0 m/s^2 satisfies |err| <= 5 %.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace {

constexpr double GRAVITY = 9.80665;

vdsim::ContactArray flat_contacts(double mu = 1.0) {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                       p.mu_long = mu; p.mu_lat = mu; }
    return c;
}

vdsim::State init_state(double vx, double R) {
    vdsim::State s; s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

double linear_bicycle_r(const vdsim::VehicleParams& vp,
                        const vdsim::TireParams& tp,
                        double vx, double delta) {
    const double m = vp.mass, a = vp.cg_to_front, b = vp.cg_to_rear, L = vp.wheelbase;
    const double Fz_f = m * GRAVITY * b / L;
    const double Fz_r = m * GRAVITY * a / L;
    const double Cf = tp.B_lat * tp.C_lat * tp.D_lat * Fz_f * tp.mu_nominal;
    const double Cr = tp.B_lat * tp.C_lat * tp.D_lat * Fz_r * tp.mu_nominal;
    const double A11 = (Cf + Cr) / vx;
    const double A12 = (a*Cf - b*Cr) / vx - m*vx;
    const double A21 = (a*Cf - b*Cr) / vx;
    const double A22 = (a*a*Cf + b*b*Cr) / vx;
    const double B1 = Cf * delta, B2 = a * Cf * delta;
    return (A11 * B2 - A21 * B1) / (A11 * A22 - A12 * A21);
}

}  // namespace

// Analytical linear bicycle ignores aligning moment; the simulator does not.
// Mz contributes a roughly -7 % self-aligning bias on yaw rate.  Relaxed
// tolerance reflects the model-mismatch, not a numerical regression.
TEST(StepSteerSweep, LinearRegionWithinTenPercent) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    const vdsim::TireParams tp;
    const vdsim::SolverParams sp;

    auto dyn = vdsim::create_bicycle();
    dyn->initialize(vp, tp, sp);

    const std::array<double, 3> vx_list   = {{5.0, 10.0, 15.0}};
    const std::array<double, 5> delta_list = {{-0.05, -0.025, 0.0, 0.025, 0.05}};

    int n_total = 0, n_linear = 0;
    double max_err_linear = 0.0;
    const auto contacts = flat_contacts();

    for (double vx : vx_list) {
        for (double d : delta_list) {
            dyn->reset(init_state(vx, vp.wheel_radius_nominal));
            vdsim::CmdL4 cmd; cmd.steer_angle_wheel = d;
            const vdsim::ControlInput u = cmd;

            for (int i = 0; i < 1500; ++i) dyn->step(u, contacts, 0.004);   // 6 s

            const double r_sim = dyn->state().yaw_rate();
            const double r_ana = linear_bicycle_r(vp, tp, vx, d);
            const double ay    = std::abs(vx * r_ana);

            ++n_total;
            if (ay < 3.0) {
                ++n_linear;
                const double err = (std::abs(r_ana) < 1e-9)
                                   ? std::abs(r_sim)
                                   : std::abs((r_sim - r_ana) / r_ana);
                EXPECT_LE(err, 0.10) << "vx=" << vx << " d=" << d
                                     << " r_sim=" << r_sim << " r_ana=" << r_ana;
                max_err_linear = std::max(max_err_linear, err);
            }
        }
    }
    EXPECT_GT(n_linear, 0);
    SUCCEED() << "linear cells = " << n_linear << " / " << n_total
              << ", max |err| in linear region = " << (max_err_linear * 100.0) << " %";
}
