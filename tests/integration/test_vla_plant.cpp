// VLA plant: CmdL1 direct torque + friction-patch ground + wheel_mu().
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

vdsim::State rolling(double vx, double R) {
    vdsim::State s;
    s.velocity.x() = vx;
    const double w = vx / R;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

}  // namespace

TEST(VlaPlant, CmdL1DirectTorqueNotThrottleMapped) {
    vdsim::VehicleParams vp;
    vp.mass = 2359.0;
    vp.plant_path = true;
    vp.drive_split_front = 0.5;
    vp.wheel_radius_nominal = 0.338;
    vp.drive_type = vdsim::VehicleParams::Drive::AWD;
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    tp.mu_nominal = 0.5;
    tp.combined_slip_enabled = true;
    vdsim::SolverParams sp;
    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, tp, sp);
    auto ground = vdsim::create_friction_patch_ground(
        0.0, 0.5, {{0.0, 500.0, 0.5}});
    dyn->reset(rolling(16.7, vp.wheel_radius_nominal));

  vdsim::CmdL1 cmd{};
  cmd.steer_angle_wheel = 0.12;
  const double fx = -15000.0;
  const double R = vp.wheel_radius_nominal;
  const double tau = 0.5 * 0.5 * fx * R;
  for (int i = 0; i < vdsim::NUM_WHEELS; ++i)
      cmd.brake_torque[i] = std::abs(tau);

  vdsim::ContactArray contacts{};
  ground->query(dyn->state(), vp, contacts);
  for (int step = 0; step < 80; ++step) {
      ground->query(dyn->state(), vp, contacts);
      dyn->step(vdsim::ControlInput{cmd}, contacts, 0.05);
  }

  const auto Fw = dyn->tire_forces_wheel();
  const auto mu = dyn->wheel_mu();
  const auto kappa = dyn->wheel_slip_ratio();
  const auto Fz = dyn->tire_Fz();
  bool saturated = false;
  bool slip = false;
  for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
      const double fxy = std::hypot(Fw[i].x(), Fw[i].y());
      const double cap = mu[i] * Fz[i];
      if (Fz[i] > 500.0 && fxy > 0.7 * cap) saturated = true;
      if (std::abs(kappa[i]) > 0.02) slip = true;
  }
  EXPECT_TRUE(saturated || slip) << "combined brake+turn should saturate or slip";
  if (saturated) {
    EXPECT_TRUE(slip);
  }
}

TEST(VlaPlant, FrictionPatchPerWheelMu) {
    vdsim::VehicleParams vp;
    vp.cg_to_front = 1.17;
    vp.cg_to_rear = 1.80;
    vp.track_front = vp.track_rear = 1.635;
    vp.wheel_radius_nominal = 0.338;
    vp.cg_height = 0.58;
    auto ground = vdsim::create_friction_patch_ground(
        0.0, 0.9, {{10.0, 30.0, 0.5}});
    vdsim::State s;
    s.position.x() = 20.0;
    s.position.y() = 0.0;
    s.velocity.x() = 10.0;
    vdsim::ContactArray c{};
    ground->query(s, vp, c);
    EXPECT_NEAR(c[vdsim::WHEEL_FL].mu_long, 0.5, 1e-9);
    EXPECT_NEAR(c[vdsim::WHEEL_RL].mu_long, 0.5, 1e-9);
}
