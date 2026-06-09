// v0.5 M1 — L5 driving on heightmap terrain.
//
// HeightmapGround already produces hub-consistent per-wheel contact (bilinear
// height + gradient normal, hub_penetration); M0 unified that with the other
// providers. This suite is the CI gap M1 closes: prove an L5 body sits on a flat
// heightmap without sinking and climbs a synthetic hill. Cliff / airborne is M2.
//
// Synthetic terrain is built in-memory (no committed .bin blobs).

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace {

// Regular grid over x in [x0, x0+(nx-1)dx], y in [y0, y0+(ny-1)dy].
struct Grid {
    int nx, ny;
    double x0, y0, dx, dy;
    std::vector<double> h;
};

// Flat heightmap at height z (constant). Exercises the heightmap query path
// rather than FlatGround.
Grid flat_grid(double z) {
    Grid g{41, 41, -20.0, -20.0, 1.0, 1.0, {}};
    g.h.assign(static_cast<std::size_t>(g.nx) * g.ny, z);
    return g;
}

// Gaussian hill centred at (xc, yc): h = h_max * exp(-(r^2)/sigma^2).
Grid hill_grid(double h_max, double xc, double yc, double sigma) {
    Grid g{91, 41, -15.0, -20.0, 1.0, 1.0, {}};
    g.h.resize(static_cast<std::size_t>(g.nx) * g.ny);
    for (int iy = 0; iy < g.ny; ++iy) {
        for (int ix = 0; ix < g.nx; ++ix) {
            const double x = g.x0 + ix * g.dx;
            const double y = g.y0 + iy * g.dy;
            const double r2 = (x - xc) * (x - xc) + (y - yc) * (y - yc);
            g.h[static_cast<std::size_t>(iy) * g.nx + ix] =
                h_max * std::exp(-r2 / (sigma * sigma));
        }
    }
    return g;
}

// Plateau at z=0 for x <= x_cliff, dropping to z_low beyond (a step-down cliff).
// Fine x spacing keeps the lip sharp so the body launches rather than ramps.
Grid cliff_grid(double x_cliff, double z_low) {
    Grid g{121, 11, -10.0, -10.0, 0.5, 2.0, {}};
    g.h.resize(static_cast<std::size_t>(g.nx) * g.ny);
    for (int iy = 0; iy < g.ny; ++iy) {
        for (int ix = 0; ix < g.nx; ++ix) {
            const double x = g.x0 + ix * g.dx;
            g.h[static_cast<std::size_t>(iy) * g.nx + ix] = (x <= x_cliff) ? 0.0 : z_low;
        }
    }
    return g;
}

std::unique_ptr<vdsim::IContactProvider> ground_from(const Grid& g, double mu) {
    return vdsim::create_heightmap_ground(g.h, g.nx, g.ny, g.x0, g.y0, g.dx, g.dy, mu);
}

vdsim::State spawn(double x, double y, double vx, double cg_z, double R) {
    vdsim::State s;
    s.position.x() = x;
    s.position.y() = y;
    s.position.z() = cg_z;
    s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

struct TerrainSetup {
    vdsim::VehicleParams vp;
    vdsim::TireParams tp;
    vdsim::SolverParams sp;
    std::unique_ptr<vdsim::IVehicleDynamics> dyn;
    std::unique_ptr<vdsim::IContactProvider> ground;

    TerrainSetup(const Grid& g, double mu) {
        vp.aero_drag_coeff = 0.0;
        tp.lugre.enabled = false;   // MF96 for deterministic CI climb
        sp.stunt_physics = true;    // L5 world-z + full gravity path
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
        dyn = vdsim::create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        ground = ground_from(g, mu);
    }

    double run(const vdsim::CmdL4& cmd, int n, double dt) {
        const vdsim::ControlInput u = cmd;
        double z_peak = dyn->state().position.z();
        for (int i = 0; i < n; ++i) {
            vdsim::ContactArray contacts;
            ground->query(dyn->state(), vp, contacts);
            dyn->step(u, contacts, dt);
            z_peak = std::max(z_peak, dyn->state().position.z());
        }
        return z_peak;
    }
};

}  // namespace

// Flat heightmap holds the body at ride height (no sink, no NaN) over 4 s coast.
TEST(Terrain, L5NoSinkOnFlat) {
    TerrainSetup h(flat_grid(0.0), 1.0);
    h.dyn->reset(spawn(0.0, 0.0, 12.0, h.vp.cg_height, h.vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd;
    h.run(cmd, 800, 0.001);                 // settle
    const double z0 = h.dyn->state().position.z();
    h.run(cmd, 3200, 0.001);
    EXPECT_TRUE(std::isfinite(z0));
    EXPECT_NEAR(h.dyn->state().position.z(), z0, 0.04);
    EXPECT_GT(z0, 0.45);
}

// Driving up a Gaussian hill raises the body well above the start ride height.
TEST(Terrain, L5ClimbsHill) {
    const Grid g = hill_grid(1.5, 30.0, 0.0, 8.0);   // ~16% max grade
    TerrainSetup h(g, 1.0);
    const double cg = h.vp.cg_height;       // start on flat region (h~0) at x=0
    h.dyn->reset(spawn(0.0, 0.0, 18.0, cg, h.vp.wheel_radius_nominal));
    const double z0 = h.dyn->state().position.z();
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.6;
    const double z_peak = h.run(cmd, 5000, 0.001);
    EXPECT_GT(h.dyn->state().position.x(), 25.0);    // actually drove onto the hill
    EXPECT_GT(z_peak, z0 + 0.3);                     // climbed >= 0.3 m
    EXPECT_TRUE(std::isfinite(z_peak));
}

// Spawned on the hill flank at the local ground height, the body stays in
// contact and stable (no sink-through, bounded attitude) over a short coast.
TEST(Terrain, L5SettlesOnHillFlank) {
    const Grid g = hill_grid(1.5, 30.0, 0.0, 8.0);
    auto probe = ground_from(g, 1.0);
    TerrainSetup h(g, 1.0);
    const double x_flank = 22.0;            // up the near side of the hill
    // local ground height at the flank for the spawn z
    vdsim::State tmp = spawn(x_flank, 0.0, 0.0, 0.0, h.vp.wheel_radius_nominal);
    vdsim::ContactArray c0;
    probe->query(tmp, h.vp, c0);
    const double z_ground = c0[vdsim::WHEEL_FL].position.z();
    h.dyn->reset(spawn(x_flank, 0.0, 0.0, z_ground + h.vp.cg_height,
                       h.vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd;
    h.run(cmd, 1500, 0.001);
    EXPECT_TRUE(std::isfinite(h.dyn->state().position.z()));
    EXPECT_LT(std::abs(h.dyn->pitch_angle_qs()), 0.5);
    const auto fz = h.dyn->tire_Fz();
    const double sum = fz[0] + fz[1] + fz[2] + fz[3];
    EXPECT_GT(sum, 0.3 * h.vp.mass * 9.80665);   // still carrying weight
}

// L5 plant on an analytic inclined plane (grade along +x, bank along +y).
struct InclinedSetup {
    vdsim::VehicleParams vp;
    vdsim::TireParams tp;
    vdsim::SolverParams sp;
    std::unique_ptr<vdsim::IVehicleDynamics> dyn;
    std::unique_ptr<vdsim::IContactProvider> ground;

    InclinedSetup(double grade, double bank, double mu) {
        vp.aero_drag_coeff = 0.0;
        tp.lugre.enabled = false;
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
        dyn = vdsim::create_stunt_dof();
        dyn->initialize(vp, tp, sp);
        ground = vdsim::create_inclined_ground(0.0, grade, bank, mu);
    }
    void run(const vdsim::CmdL4& cmd, int n, double dt) {
        const vdsim::ControlInput u = cmd;
        for (int i = 0; i < n; ++i) {
            vdsim::ContactArray c;
            ground->query(dyn->state(), vp, c);
            dyn->step(u, c, dt);
        }
    }
};

// Coasting up a grade bleeds more speed than flat (gravity component along -x).
TEST(Terrain, L5CoastUphillSlows) {
    vdsim::CmdL4 cmd;
    InclinedSetup flat(0.0, 0.0, 1.0);
    flat.dyn->reset(spawn(0.0, 0.0, 18.0, flat.vp.cg_height, flat.vp.wheel_radius_nominal));
    flat.run(cmd, 2500, 0.001);
    const double vx_flat = flat.dyn->state().velocity.x();

    InclinedSetup hill(0.10, 0.0, 1.0);
    hill.dyn->reset(spawn(0.0, 0.0, 18.0, hill.vp.cg_height, hill.vp.wheel_radius_nominal));
    hill.run(cmd, 2500, 0.001);
    EXPECT_LT(hill.dyn->state().velocity.x(), vx_flat * 0.92);
}

// On a banked plane the body settles toward the surface normal -> non-trivial
// roll attitude with the bank's sign (bank>0 rises toward +y).
TEST(Terrain, L5BankInducesRoll) {
    const double bank = 0.12;   // ~6.9 deg
    InclinedSetup h(0.0, bank, 1.0);
    h.dyn->reset(spawn(0.0, 0.0, 6.0, h.vp.cg_height, h.vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd;
    h.run(cmd, 1500, 0.001);
    const double roll = h.dyn->roll_angle_qs();
    EXPECT_TRUE(std::isfinite(roll));
    EXPECT_GT(std::abs(roll), 0.3 * bank);   // body leans with the bank
    EXPECT_LT(std::abs(roll), 2.0 * bank);
}

// contact -> Fz ~ 0, no mid-air phantom load), then lands on the lower plateau.
TEST(Terrain, L5BriefAirOverCliff) {
    const double x_cliff = 15.0, z_low = -1.5;
    TerrainSetup h(cliff_grid(x_cliff, z_low), 1.0);
    h.dyn->reset(spawn(0.0, 0.0, 24.0, h.vp.cg_height, h.vp.wheel_radius_nominal));
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.1;                  // hold approach speed against rolling drag
    const vdsim::ControlInput u = cmd;
    const double mg = h.vp.mass * 9.80665;
    bool saw_air = false;
    double max_individual_in_air = 0.0;
    for (int i = 0; i < 2000; ++i) {
        vdsim::ContactArray contacts;
        h.ground->query(h.dyn->state(), h.vp, contacts);
        h.dyn->step(u, contacts, 0.001);
        const double x = h.dyn->state().position.x();
        const auto fz = h.dyn->tire_Fz();
        const double sum = fz[0] + fz[1] + fz[2] + fz[3];
        if (x > x_cliff + 1.0 && sum < 0.05 * mg) {
            saw_air = true;
            for (int w = 0; w < vdsim::NUM_WHEELS; ++w)
                max_individual_in_air = std::max(max_individual_in_air, fz[w]);
        }
    }
    EXPECT_TRUE(saw_air);                              // there was an airborne interval
    EXPECT_LT(max_individual_in_air, 0.05 * mg);       // no mid-air phantom Fz
    EXPECT_TRUE(std::isfinite(h.dyn->state().position.z()));
    EXPECT_LT(h.dyn->state().position.z(), z_low + h.vp.cg_height + 0.3);  // landed low
    const auto fz_end = h.dyn->tire_Fz();
    EXPECT_GT(fz_end[0] + fz_end[1] + fz_end[2] + fz_end[3], 0.2 * mg);    // back in contact
}

// M5b — banked circular turn (CurvedGround). At the neutral speed
// v_n = sqrt(g R tan(bank)) with Ackermann steer for radius R, the body corners
// around the centre and holds the line (radius stays near R, no slide-off).
TEST(Terrain, BankedTurnHoldsLine) {
    const double R = 40.0, bank = 0.20;
    const double v_n = std::sqrt(9.80665 * R * std::tan(bank));   // ~8.9 m/s
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    auto dyn = vdsim::create_stunt_dof();
    dyn->initialize(vp, tp, sp);
    auto ground = vdsim::create_curved_ground(0.0, 0.0, R, bank, 0.0, 1.0);

    vdsim::State s;                            // spawn at (R,0) heading +y (CCW tangent)
    s.position = vdsim::Vec3(R, 0.0, vp.cg_height);
    s.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, M_PI / 2});
    s.velocity.x() = v_n;
    const double w = v_n / vp.wheel_radius_nominal;
    s.wheel_spin = {{w, w, w, w}};
    dyn->reset(s);

    vdsim::CmdL4 cmd;
    cmd.steer_angle_wheel = std::atan(vp.wheelbase / R);   // Ackermann for radius R (left)
    const vdsim::ControlInput u = cmd;
    const double yaw0 = dyn->state().yaw();
    double rho_min = R, rho_max = R;
    for (int i = 0; i < 2500; ++i) {
        vdsim::ContactArray c;
        ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 0.001);
        const auto& p = dyn->state().position;
        const double rho = std::hypot(p.x(), p.y());
        rho_min = std::min(rho_min, rho);
        rho_max = std::max(rho_max, rho);
    }
    const double dyaw = dyn->state().yaw() - yaw0;
    EXPECT_TRUE(std::isfinite(dyn->state().position.z()));
    EXPECT_GT(std::abs(dyaw), 0.3) << "should corner around the turn";
    EXPECT_GT(rho_min, R - 8.0) << "rho_min=" << rho_min;     // didn't cut far inboard
    EXPECT_LT(rho_max, R + 12.0) << "rho_max=" << rho_max;    // didn't slide off outboard
}
