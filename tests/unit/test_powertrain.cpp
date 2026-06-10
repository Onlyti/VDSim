// D1 — 2D engine torque map bilinear interpolation + domain clamping.

#include "vdsim/params.hpp"
#include "vdsim/powertrain.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

vdsim::EngineMap sample_map() {
    vdsim::EngineMap m;
    m.rpm_breaks      = {1000, 3000, 6000};
    m.throttle_breaks = {0.0, 1.0};
    m.torque = {
        {-20, -30, -40},     // closed throttle -> engine braking (negative)
        {180, 320, 240},     // WOT
    };
    return m;
}

}  // namespace

TEST(EngineMap, ExactBreakpoints) {
    const auto m = sample_map();
    EXPECT_DOUBLE_EQ(m.eval(3000, 1.0), 320.0);
    EXPECT_DOUBLE_EQ(m.eval(1000, 0.0), -20.0);
    EXPECT_DOUBLE_EQ(m.eval(6000, 1.0), 240.0);
}

TEST(EngineMap, BilinearMidpoints) {
    const auto m = sample_map();
    // mid rpm (2000) at WOT: between 180 and 320 -> 250
    EXPECT_NEAR(m.eval(2000, 1.0), 250.0, 1e-9);
    // half throttle at 3000 rpm: between -30 and 320 -> 145
    EXPECT_NEAR(m.eval(3000, 0.5), 145.0, 1e-9);
    // mid rpm + half throttle: bilinear of corners (-20,-30 / 180,320) at (0.5,0.5)
    // rpm 2000 row0 = -25, row1 = 250 -> throttle 0.5 -> 112.5
    EXPECT_NEAR(m.eval(2000, 0.5), 112.5, 1e-9);
}

TEST(EngineMap, ClampsOutsideDomain) {
    const auto m = sample_map();
    EXPECT_DOUBLE_EQ(m.eval(500,  1.0), m.eval(1000, 1.0));   // below first rpm break
    EXPECT_DOUBLE_EQ(m.eval(9000, 1.0), m.eval(6000, 1.0));   // above redline break
    EXPECT_DOUBLE_EQ(m.eval(3000, 1.5), m.eval(3000, 1.0));   // throttle clamp
    EXPECT_DOUBLE_EQ(m.eval(3000, -0.2), m.eval(3000, 0.0));
}

TEST(EngineMap, ClosedThrottleIsEngineBraking) {
    const auto m = sample_map();
    for (double rpm : {1000.0, 3000.0, 6000.0})
        EXPECT_LT(m.eval(rpm, 0.0), 0.0) << "closed throttle must motor (negative torque)";
}

TEST(EngineMap, InvalidMapReturnsZero) {
    vdsim::EngineMap m;            // empty
    EXPECT_FALSE(m.valid());
    EXPECT_DOUBLE_EQ(m.eval(3000, 1.0), 0.0);
}

namespace {
std::filesystem::path write_yaml(const std::string& body) {
    const auto p = std::filesystem::temp_directory_path() /
        ("vdsim_pt_" + std::to_string(::getpid()) + std::to_string(rand()) + ".yaml");
    std::ofstream(p) << body;
    return p;
}
}  // namespace

TEST(PowertrainYaml, DefaultDisabled) {
    const auto p = write_yaml("mass: 1500\nwheelbase: 2.7\ncg_to_front: 1.2\ncg_to_rear: 1.5\n");
    const auto vp = vdsim::VehicleParams::from_yaml(p.string());
    std::filesystem::remove(p);
    EXPECT_FALSE(vp.powertrain.enabled) << "no powertrain block -> legacy flat torque";
}

TEST(PowertrainYaml, ParsesEngineGearboxShift) {
    const auto p = write_yaml(
        "mass: 1500\nwheelbase: 2.7\ncg_to_front: 1.2\ncg_to_rear: 1.5\n"
        "powertrain:\n"
        "  engine:\n"
        "    idle_rpm: 850\n"
        "    redline_rpm: 6800\n"
        "    inertia: 0.22\n"
        "    rpm_breaks: [1000, 3000, 6000]\n"
        "    throttle_breaks: [0.0, 1.0]\n"
        "    torque_map:\n"
        "      - [-20, -30, -40]\n"
        "      - [180, 320, 240]\n"
        "  gearbox:\n"
        "    gear_ratios: [3.2, 1.9, 1.3, 1.0, 0.8, 0.65]\n"
        "    final_drive: 3.9\n"
        "    efficiency: 0.9\n"
        "    shift_time: 0.25\n"
        "  shift:\n"
        "    mode: auto_rpm\n"
        "    upshift_rpm: 6000\n"
        "    downshift_rpm: 2000\n");
    const auto vp = vdsim::VehicleParams::from_yaml(p.string());
    std::filesystem::remove(p);

    ASSERT_TRUE(vp.powertrain.enabled);
    EXPECT_DOUBLE_EQ(vp.powertrain.engine.idle_rpm, 850.0);
    EXPECT_DOUBLE_EQ(vp.powertrain.engine.redline_rpm, 6800.0);
    ASSERT_TRUE(vp.powertrain.engine.map.valid());
    EXPECT_DOUBLE_EQ(vp.powertrain.engine.map.eval(3000, 1.0), 320.0);
    EXPECT_LT(vp.powertrain.engine.map.eval(3000, 0.0), 0.0);
    EXPECT_EQ(vp.powertrain.gearbox.gear_ratios.size(), 6u);
    EXPECT_DOUBLE_EQ(vp.powertrain.gearbox.final_drive, 3.9);
    EXPECT_DOUBLE_EQ(vp.powertrain.gearbox.shift_time, 0.25);
    EXPECT_EQ(vp.powertrain.shift_mode, vdsim::PowertrainParams::ShiftMode::AutoRpmThreshold);
    EXPECT_DOUBLE_EQ(vp.powertrain.upshift_rpm, 6000.0);
}
