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

namespace {
vdsim::PowertrainParams make_pt() {
    vdsim::PowertrainParams p;
    p.enabled = true;
    p.engine.idle_rpm = 800; p.engine.redline_rpm = 6500; p.engine.inertia = 0.25;
    p.engine.map.rpm_breaks = {1000, 3000, 6000};
    p.engine.map.throttle_breaks = {0.0, 1.0};
    p.engine.map.torque = {{-20, -30, -40}, {180, 320, 240}};
    p.gearbox.gear_ratios = {3.5, 2.0, 1.3, 1.0, 0.8};
    p.gearbox.final_drive = 4.0; p.gearbox.efficiency = 0.9; p.gearbox.shift_time = 0.3;
    return p;
}
}  // namespace

TEST(EngineGearbox, RpmCouplesToWheelSpeed) {
    vdsim::EngineGearbox eg; eg.configure(make_pt());
    const double omega = 10.0;       // rad/s driven wheel
    eg.axle_torque(/*throttle=*/0.5, /*brake=*/0.0, omega, /*v=*/3.0, /*gear=*/1, 1e-3);
    const double expect = omega * 3.5 * 4.0 * 60.0 / (2.0 * M_PI);  // ratio*fd
    EXPECT_NEAR(eg.engine_rpm(), expect, 1.0);
}

TEST(EngineGearbox, AxleTorqueScalesByRatioFdEff) {
    vdsim::EngineGearbox eg; eg.configure(make_pt());
    const double omega = 30.0;       // -> rpm well above idle, locked
    const double T = eg.axle_torque(1.0, 0.0, omega, 9.0, 1, 1e-3);
    const double Te = make_pt().engine.map.eval(eg.engine_rpm(), 1.0);
    EXPECT_NEAR(T, Te * 3.5 * 4.0 * 0.9, 1e-6);
}

TEST(EngineGearbox, ReflectedInertiaIsGearDependent) {
    auto p = make_pt();
    vdsim::EngineGearbox lo; p.start_gear = 1; lo.configure(p);
    vdsim::EngineGearbox hi; p.start_gear = 5; hi.configure(p);
    EXPECT_GT(lo.reflected_inertia(), hi.reflected_inertia());
    EXPECT_NEAR(lo.reflected_inertia(), 0.25 * std::pow(3.5 * 4.0, 2), 1e-9);
}

TEST(EngineGearbox, AutoUpshiftAtThreshold) {
    auto p = make_pt();
    p.shift_mode = vdsim::PowertrainParams::ShiftMode::AutoRpmThreshold;
    p.upshift_rpm = 5000; p.downshift_rpm = 1500;
    vdsim::EngineGearbox eg; eg.configure(p);
    // omega that puts gear-1 rpm above 5000: rpm = omega*3.5*4*9.549
    const double omega = 5200.0 / (3.5 * 4.0 * 60.0 / (2.0 * M_PI));
    eg.axle_torque(1.0, 0.0, omega, 12.0, /*manual gear ignored*/1, 1e-3);
    EXPECT_EQ(eg.current_gear(), 2) << "should upshift above the threshold";
}

TEST(EngineGearbox, ShiftTimeInterruptsTorque) {
    auto p = make_pt();
    p.shift_mode = vdsim::PowertrainParams::ShiftMode::AutoRpmThreshold;
    p.upshift_rpm = 5000; p.gearbox.shift_time = 0.3;
    vdsim::EngineGearbox eg; eg.configure(p);
    const double omega = 5200.0 / (3.5 * 4.0 * 60.0 / (2.0 * M_PI));
    const double T_shift = eg.axle_torque(1.0, 0.0, omega, 12.0, 1, 1e-3);  // triggers shift
    EXPECT_DOUBLE_EQ(T_shift, 0.0) << "clutch open during the shift";
}

TEST(EngineGearbox, CustomShiftPolicyIsUsed) {
    vdsim::EngineGearbox eg; eg.configure(make_pt());
    bool called = false;
    eg.set_shift_policy([&](const vdsim::ShiftContext& c) {
        called = true;
        EXPECT_EQ(c.num_gears, 5);
        return 4;                    // force 4th gear
    });
    eg.axle_torque(0.6, 0.0, 40.0, 12.0, 1, 1e-3);
    EXPECT_TRUE(called);
    EXPECT_EQ(eg.current_gear(), 4);
}

TEST(EngineGearbox, IdleFloorAndNoReverseCreepAtStandstill) {
    vdsim::EngineGearbox eg; eg.configure(make_pt());
    const double T_closed = eg.axle_torque(0.0, 0.0, 0.0, 0.0, 1, 1e-3);
    EXPECT_GE(eg.engine_rpm(), 800.0) << "idle floor at standstill";
    EXPECT_GE(T_closed, 0.0) << "slipping launch clutch does not push backward";
    const double T_open = eg.axle_torque(1.0, 0.0, 0.0, 0.0, 1, 1e-3);
    EXPECT_GT(T_open, 0.0) << "throttle launches forward";
}

TEST(EngineGearbox, EngineBrakingWhenCoastingLocked) {
    vdsim::EngineGearbox eg; eg.configure(make_pt());
    const double omega = 60.0;       // locked, high rpm
    const double T = eg.axle_torque(0.0, 0.0, omega, 18.0, 1, 1e-3);
    EXPECT_LT(T, 0.0) << "closed throttle while rolling -> engine braking (negative axle torque)";
}
