// Per-model declaration of which raw-log channels the dynamics really models
// (IVehicleDynamics::models_channel), checked against what the state does.
//
// The trace writer trusts this declaration, so it is measured here instead of
// being derived from level(): a model that declares a channel modeled must move
// it under a lateral maneuver, a model that does not must leave it at zero.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace {

using MC = vdsim::IVehicleDynamics::ModeledChannel;

struct Peaks {
    double roll_rate {0.0};
    double pitch_rate {0.0};
    double stroke {0.0};
};

vdsim::State level_on_flat(double vx, const vdsim::VehicleParams& vp) {
    vdsim::State s;
    s.position.z() = vp.cg_height;
    s.velocity.x() = vx;
    const double w = vx / vp.wheel_radius_nominal;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

// Steer ramp at 15 m/s, then hold; returns the peak |roll rate|, |pitch rate|
// and |susp_compression| seen on the published state.
Peaks lateral_maneuver(vdsim::IVehicleDynamics& dyn, const vdsim::VehicleParams& vp) {
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    dyn.reset(level_on_flat(15.0, vp));
    Peaks p;
    const double dt = 0.001;
    for (int i = 0; i < 2500; ++i) {
        vdsim::CmdL4 cmd;
        cmd.steer_angle_wheel = 0.05 * std::min(1.0, i * dt / 0.3);
        vdsim::ContactArray contacts;
        ground->query(dyn.state(), vp, contacts);
        dyn.step(vdsim::ControlInput{cmd}, contacts, dt);
        const auto& s = dyn.state();
        p.roll_rate  = std::max(p.roll_rate,  std::abs(s.angular_velocity.x()));
        p.pitch_rate = std::max(p.pitch_rate, std::abs(s.angular_velocity.y()));
        for (double c : s.susp_compression) p.stroke = std::max(p.stroke, std::abs(c));
    }
    return p;
}

std::unique_ptr<vdsim::IVehicleDynamics> make(
    std::unique_ptr<vdsim::IVehicleDynamics> dyn,
    const vdsim::VehicleParams& vp, bool l5 = false, bool spatial = false) {
    vdsim::TireParams tp;
    tp.lugre.enabled = false;
    vdsim::SolverParams sp;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    if (l5) {
        sp.stunt_physics = true;
        sp.l5_spatial_suspension = spatial;
    }
    dyn->initialize(vp, tp, sp);
    return dyn;
}

}  // namespace

TEST(ChannelModeling, DeclarationsFollowTheModelNotTheLevelNumber) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;

    auto lk = make(vdsim::create_kinematic(), vp);
    auto l1 = make(vdsim::create_bicycle(), vp);
    auto l2 = make(vdsim::create_seven_dof(), vp);
    auto l3 = make(vdsim::create_fourteen_dof(), vp);
    auto l4 = make(vdsim::create_fourteen_dof_kinematic(), vp);
    auto l5_off = make(vdsim::create_stunt_dof(), vp, true, false);
    auto l5_on  = make(vdsim::create_stunt_dof(), vp, true, true);

    for (auto* d : {lk.get(), l1.get(), l2.get()}) {
        EXPECT_FALSE(d->models_channel(MC::RollPitchRate));
        EXPECT_FALSE(d->models_channel(MC::WheelTravel));
    }
    for (auto* d : {l3.get(), l4.get()}) {
        EXPECT_TRUE(d->models_channel(MC::RollPitchRate));
        EXPECT_TRUE(d->models_channel(MC::WheelTravel));
    }
    EXPECT_TRUE(l5_off->models_channel(MC::RollPitchRate));
    EXPECT_FALSE(l5_off->models_channel(MC::WheelTravel));
    EXPECT_TRUE(l5_on->models_channel(MC::RollPitchRate));
    EXPECT_TRUE(l5_on->models_channel(MC::WheelTravel));
}

TEST(ChannelModeling, NotModeledModelsLeaveTheChannelsAtZero) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    auto l1 = make(vdsim::create_bicycle(), vp);
    auto l2 = make(vdsim::create_seven_dof(), vp);
    for (auto* d : {l1.get(), l2.get()}) {
        const Peaks p = lateral_maneuver(*d, vp);
        EXPECT_EQ(p.roll_rate, 0.0);
        EXPECT_EQ(p.pitch_rate, 0.0);
        EXPECT_EQ(p.stroke, 0.0);
    }
}

// The rate comes from the integrated roll/pitch states, so it must be
// nonzero once the maneuver excites them.
TEST(ChannelModeling, L3PublishesIntegratedRollAndPitchRate) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    auto l3 = make(vdsim::create_fourteen_dof(), vp);
    const Peaks p = lateral_maneuver(*l3, vp);
    EXPECT_GT(p.roll_rate, 1e-4);
    EXPECT_GT(p.stroke, 1e-5);
}

TEST(ChannelModeling, L4PublishesRollAndPitchRate) {
    const std::string kin = std::string(VDSIM_SOURCE_DIR)
        + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    auto l4 = make(vdsim::create_fourteen_dof_kinematic(), vp);
    auto topo = vdsim::mb::SuspensionTopology::from_yaml(kin);
    ASSERT_TRUE(vdsim::mb::attach_topology_front(*l4, topo));
    const Peaks p = lateral_maneuver(*l4, vp);
    EXPECT_GT(p.roll_rate, 1e-4);
    EXPECT_GT(p.stroke, 1e-5);
}

TEST(ChannelModeling, L5RateIsLiveAndStrokeExistsOnlyWithSpatialSuspension) {
    vdsim::VehicleParams vp;
    vp.aero_drag_coeff = 0.0;
    auto off = make(vdsim::create_stunt_dof(), vp, true, false);
    auto on  = make(vdsim::create_stunt_dof(), vp, true, true);
    const Peaks p_off = lateral_maneuver(*off, vp);
    const Peaks p_on  = lateral_maneuver(*on, vp);
    EXPECT_GT(p_off.roll_rate, 1e-4);
    EXPECT_GT(p_on.roll_rate, 1e-4);
    EXPECT_EQ(p_off.stroke, 0.0);
    EXPECT_GT(p_on.stroke, 1e-5);
}
