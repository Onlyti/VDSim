// MW6 — user subsystem-module plugin: load a .so, install it on a model, run it.
//
// The sample plugin examples/modules/custom_brake.cpp is built into
// ${VDSIM_TEST_MODULES_DIR}/custom_brake.so by the examples CMake.

#include <string>

#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/module_plugin.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

namespace {
std::string mod_path(const char* file) {
    return std::string(VDSIM_TEST_MODULES_DIR) + "/" + file;
}
vdsim::State rolling(double v) {
    vdsim::State s;
    s.velocity.x() = v;
    const double w = v / 0.32;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}
}  // namespace

TEST(ModulePlugin, LoadsAndReportsKind) {
    auto m = vdsim::load_module_plugin(mod_path("custom_brake.so"));
    ASSERT_TRUE(m.ok) << m.error;
    EXPECT_EQ(m.kind, vdsim::ModuleKind::Brake);
    EXPECT_EQ(m.name, "custom_brake");
    ASSERT_NE(m.brake, nullptr);
}

TEST(ModulePlugin, MissingFileFailsCleanly) {
    auto m = vdsim::load_module_plugin(mod_path("does_not_exist.so"));
    EXPECT_FALSE(m.ok);
    EXPECT_FALSE(m.error.empty());
}

TEST(ModulePlugin, InstalledModuleBrakes) {
    const vdsim::VehicleParams vp;
    auto m = vdsim::load_module_plugin(mod_path("custom_brake.so"));
    ASSERT_TRUE(m.ok) << m.error;

    auto dyn = vdsim::create_seven_dof();
    dyn->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
    ASSERT_TRUE(vdsim::install_module(*dyn, m)) << "L2 must accept a brake module";
    dyn->reset(rolling(25.0));

    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd;
    cmd.throttle = 0.0;
    cmd.brake    = 1.0;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < 800; ++i) {
        vdsim::ContactArray c;
        ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 1e-3);
    }
    EXPECT_LT(dyn->state().vx(), 24.0) << "the installed brake module should decelerate the car";
}
