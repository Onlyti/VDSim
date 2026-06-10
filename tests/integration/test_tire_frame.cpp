// Tire-frame vs body-frame per-wheel force accessors.
//
// The model computes tire forces in the WHEEL frame (slip alpha/kappa are defined
// there) and rotates them by the steer angle into the body frame for the EOM.
// tire_forces_wheel() exposes the un-rotated wheel-frame force; tire_forces_body()
// the rotated one. For a steered front wheel they must differ by exactly the steer
// rotation (same magnitude); for an unsteered rear wheel they must be identical.

#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

vdsim::State rolling(double Vx) {
    vdsim::State s;
    s.velocity.x() = Vx;
    const double w = Vx / 0.32;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

double ang(const vdsim::Vec3& f) { return std::atan2(f.y(), f.x()); }

}  // namespace

TEST(TireFrame, WheelForceIsBodyForceUnrotatedBySteer) {
    vdsim::VehicleParams vp;          // default; ackerman 0 -> both front wheels steer by delta
    vdsim::TireParams tp;
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    dyn->reset(rolling(15.0));

    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    const double delta = 0.12;        // road-wheel steer [rad]
    vdsim::CmdL4 cmd; cmd.steer_angle_wheel = delta;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 2000; ++i) {  // settle to steady cornering
        vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 1e-3);
    }

    const auto wheel = dyn->tire_forces_wheel();
    const auto body  = dyn->tire_forces_body();

    // Rotation preserves magnitude on every wheel.
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i)
        EXPECT_NEAR(wheel[i].norm(), body[i].norm(), 1e-6 + 1e-6 * body[i].norm())
            << "wheel " << i << " frames must be a pure rotation of each other";

    // Front wheels are steered: body != wheel, and the rotation angle == steer.
    for (int i : {vdsim::WHEEL_FL, vdsim::WHEEL_FR}) {
        ASSERT_GT(wheel[i].norm(), 100.0) << "front wheel should carry real force";
        EXPECT_GT((body[i] - wheel[i]).norm(), 1.0) << "front body must differ from wheel under steer";
        double d = ang(body[i]) - ang(wheel[i]);
        d = std::atan2(std::sin(d), std::cos(d));   // wrap to (-pi, pi]
        EXPECT_NEAR(d, delta, 1e-3) << "front body/wheel rotation must equal the steer angle";
    }

    // Rear wheels are not steered: body == wheel.
    for (int i : {vdsim::WHEEL_RL, vdsim::WHEEL_RR}) {
        EXPECT_NEAR((body[i] - wheel[i]).norm(), 0.0, 1e-6 + 1e-6 * body[i].norm())
            << "rear wheel " << i << " has no steer rotation";
    }
}
