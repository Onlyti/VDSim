// U5 — user-defined subsystem modules.
//
// A custom module (a C++ subclass of IBrakeSystem/ISteeringSystem/IDrivetrain/
// ISuspension/IAntiRollBar) can be installed via IVehicleDynamics::set_*_module and
// the plant calls into it. Three properties are checked:
//   1. Transparency — a wrapper that delegates to the built-in default reproduces the
//      baseline trajectory (the injection plumbing adds nothing).
//   2. Effect — a custom law changes the trajectory as expected.
//   3. Scoping — brake/steering/drivetrain live on L2/L3, suspension/ARB on L3 only,
//      and L1 hosts none; null and out-of-range installs are rejected.

#include "vdsim/default_subsystems.hpp"
#include "vdsim/control.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

vdsim::State rolling(double Vx) {
    vdsim::State s;
    s.velocity.x() = Vx;
    const double w = Vx / 0.32;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

// ---- Transparent wrappers: delegate every call to a fresh built-in default. ----
struct WrapBrake final : vdsim::IBrakeSystem {
    std::unique_ptr<vdsim::IBrakeSystem> inner;
    explicit WrapBrake(const vdsim::VehicleParams& vp)
        : inner(vdsim::make_default_brake(vp, vp.brake_deadtime_s)) {}
    std::array<double, vdsim::NUM_WHEELS> wheel_torque(const vdsim::SubsystemContext& c) override {
        return inner->wheel_torque(c);
    }
    void begin_step(const vdsim::SubsystemContext& c, double dt) override { inner->begin_step(c, dt); }
    void reset() override { inner->reset(); }
};
struct WrapSteering final : vdsim::ISteeringSystem {
    std::unique_ptr<vdsim::ISteeringSystem> inner;
    explicit WrapSteering(const vdsim::VehicleParams& vp)
        : inner(vdsim::make_default_steering(vp, vp.steer_deadtime_s)) {}
    vdsim::SteeringOutput apply(const vdsim::SubsystemContext& c) override { return inner->apply(c); }
    void begin_step(const vdsim::SubsystemContext& c, double dt) override { inner->begin_step(c, dt); }
    void reset() override { inner->reset(); }
};
struct WrapDrivetrain final : vdsim::IDrivetrain {
    std::unique_ptr<vdsim::IDrivetrain> inner;
    explicit WrapDrivetrain(const vdsim::VehicleParams& vp)
        : inner(vdsim::make_default_drivetrain(vp, vp.drive_deadtime_s)) {}
    vdsim::DrivetrainOutput apply(const vdsim::SubsystemContext& c) override { return inner->apply(c); }
    void begin_step(const vdsim::SubsystemContext& c, double dt) override { inner->begin_step(c, dt); }
    void reset() override { inner->reset(); }
    double wheel_engine_inertia(int w) const override { return inner->wheel_engine_inertia(w); }
};
struct WrapSuspension final : vdsim::ISuspension {
    std::unique_ptr<vdsim::ISuspension> inner;
    explicit WrapSuspension(const vdsim::VehicleParams& vp)
        : inner(vdsim::make_default_suspension(vp)) {}
    double force(const vdsim::SubsystemContext& c, const vdsim::CornerInput& ci) override {
        return inner->force(c, ci);
    }
    void reset() override { inner->reset(); }
};
struct WrapARB final : vdsim::IAntiRollBar {
    std::unique_ptr<vdsim::IAntiRollBar> inner;
    WrapARB(const vdsim::VehicleParams& vp, int axle)
        : inner(vdsim::make_default_antirollbar(vp, axle)) {}
    std::pair<double, double> force(const vdsim::SubsystemContext& c, const vdsim::AxleDefl& d) override {
        return inner->force(c, d);
    }
    void reset() override { inner->reset(); }
};

// A custom brake returning a signed torque (opposing spin) scaled by `gain`.
struct ScaledBrake final : vdsim::IBrakeSystem {
    vdsim::VehicleParams vp; double gain; int calls{0}; int begins{0};
    ScaledBrake(const vdsim::VehicleParams& v, double g) : vp(v), gain(g) {}
    void begin_step(const vdsim::SubsystemContext&, double) override { ++begins; }
    std::array<double, vdsim::NUM_WHEELS> wheel_torque(const vdsim::SubsystemContext& c) override {
        ++calls;
        std::array<double, vdsim::NUM_WHEELS> Tb{};
        const double per_wheel = 0.25 * gain * c.cmd.brake * vp.max_brake_torque;
        for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
            const double s = c.state.wheel_spin[i];
            Tb[i] = -(s >= 0.0 ? 1.0 : -1.0) * per_wheel;
        }
        return Tb;
    }
};

struct EndState { double x, y, vx, vy, yaw, r; };

template <class Setup>
EndState drive(vdsim::IVehicleDynamics& dyn, const vdsim::VehicleParams& vp,
          double Vx0, double throttle, double brake, double steer, int n, Setup setup) {
    dyn.initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
    dyn.reset(rolling(Vx0));
    setup(dyn);
    auto ground = vdsim::create_flat_ground(0.0, 1.0);
    vdsim::CmdL4 cmd; cmd.throttle = throttle; cmd.brake = brake; cmd.steer_angle_wheel = steer;
    const vdsim::ControlInput u = cmd;
    for (int i = 0; i < n; ++i) {
        vdsim::ContactArray c; ground->query(dyn.state(), vp, c);
        dyn.step(u, c, 1e-3);
    }
    const auto& s = dyn.state();
    return {s.position.x(), s.position.y(), s.vx(), s.vy(), s.yaw(), s.yaw_rate()};
}

auto no_setup = [](vdsim::IVehicleDynamics&) {};

}  // namespace

// 1. Transparency: wrapping the defaults reproduces the baseline (L2).
TEST(UserModules, WrapperIsTransparentL2) {
    vdsim::VehicleParams vp;
    auto base = vdsim::create_seven_dof();
    const EndState rb = drive(*base, vp, 20.0, 0.3, 0.1, 0.05, 1500, no_setup);

    auto wrapped = vdsim::create_seven_dof();
    const EndState rw = drive(*wrapped, vp, 20.0, 0.3, 0.1, 0.05, 1500,
        [&](vdsim::IVehicleDynamics& d) {
            ASSERT_TRUE(d.set_brake_module(std::make_shared<WrapBrake>(vp)));
            ASSERT_TRUE(d.set_steering_module(std::make_shared<WrapSteering>(vp)));
            ASSERT_TRUE(d.set_drivetrain_module(std::make_shared<WrapDrivetrain>(vp)));
        });

    EXPECT_NEAR(rw.x,   rb.x,   1e-9);
    EXPECT_NEAR(rw.y,   rb.y,   1e-9);
    EXPECT_NEAR(rw.vx,  rb.vx,  1e-9);
    EXPECT_NEAR(rw.vy,  rb.vy,  1e-9);
    EXPECT_NEAR(rw.yaw, rb.yaw, 1e-9);
}

// 1b. Transparency on L3 incl. suspension + both ARBs.
TEST(UserModules, WrapperIsTransparentL3) {
    vdsim::VehicleParams vp;
    auto base = vdsim::create_fourteen_dof();
    const EndState rb = drive(*base, vp, 22.0, 0.2, 0.0, 0.08, 1500, no_setup);

    auto wrapped = vdsim::create_fourteen_dof();
    const EndState rw = drive(*wrapped, vp, 22.0, 0.2, 0.0, 0.08, 1500,
        [&](vdsim::IVehicleDynamics& d) {
            ASSERT_TRUE(d.set_brake_module(std::make_shared<WrapBrake>(vp)));
            ASSERT_TRUE(d.set_suspension_module(std::make_shared<WrapSuspension>(vp)));
            ASSERT_TRUE(d.set_antirollbar_module(0, std::make_shared<WrapARB>(vp, 0)));
            ASSERT_TRUE(d.set_antirollbar_module(1, std::make_shared<WrapARB>(vp, 1)));
        });

    EXPECT_NEAR(rw.x,   rb.x,   1e-9);
    EXPECT_NEAR(rw.y,   rb.y,   1e-9);
    EXPECT_NEAR(rw.vx,  rb.vx,  1e-9);
    EXPECT_NEAR(rw.r,   rb.r,   1e-9);
}

// 2. Effect + call cadence: a stronger brake decelerates more; begin_step once per
//    step() and wheel_torque once per RK4 stage (strictly more often).
TEST(UserModules, CustomBrakeStrengthAndCadence) {
    vdsim::VehicleParams vp;
    const int N = 800;

    auto weak = vdsim::create_seven_dof();
    auto weak_mod = std::make_shared<ScaledBrake>(vp, 1.0);
    const EndState rw = drive(*weak, vp, 25.0, 0.0, 1.0, 0.0, N,
        [&](vdsim::IVehicleDynamics& d) { ASSERT_TRUE(d.set_brake_module(weak_mod)); });

    auto strong = vdsim::create_seven_dof();
    auto strong_mod = std::make_shared<ScaledBrake>(vp, 2.0);
    const EndState rs = drive(*strong, vp, 25.0, 0.0, 1.0, 0.0, N,
        [&](vdsim::IVehicleDynamics& d) { ASSERT_TRUE(d.set_brake_module(strong_mod)); });

    EXPECT_LT(rs.vx, rw.vx) << "stronger brake gain should leave a lower speed";
    EXPECT_EQ(weak_mod->begins, N) << "begin_step runs once per step()";
    EXPECT_GT(weak_mod->calls, weak_mod->begins) << "wheel_torque runs per RK4 stage";
}

// 3. Scoping: which levels host which modules; null / bad-axle rejected.
TEST(UserModules, InstallScoping) {
    vdsim::VehicleParams vp;
    auto l1 = vdsim::create_bicycle();
    auto l2 = vdsim::create_seven_dof();
    auto l3 = vdsim::create_fourteen_dof();
    for (auto* d : {l1.get(), l2.get(), l3.get()})
        d->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});

    // L1 bicycle hosts no subsystem modules.
    EXPECT_FALSE(l1->set_brake_module(std::make_shared<WrapBrake>(vp)));
    EXPECT_FALSE(l1->set_suspension_module(std::make_shared<WrapSuspension>(vp)));

    // L2 planar: actuators yes, vertical/roll no.
    EXPECT_TRUE(l2->set_brake_module(std::make_shared<WrapBrake>(vp)));
    EXPECT_TRUE(l2->set_steering_module(std::make_shared<WrapSteering>(vp)));
    EXPECT_TRUE(l2->set_drivetrain_module(std::make_shared<WrapDrivetrain>(vp)));
    EXPECT_FALSE(l2->set_suspension_module(std::make_shared<WrapSuspension>(vp)));
    EXPECT_FALSE(l2->set_antirollbar_module(0, std::make_shared<WrapARB>(vp, 0)));

    // L3 ride: all five.
    EXPECT_TRUE(l3->set_brake_module(std::make_shared<WrapBrake>(vp)));
    EXPECT_TRUE(l3->set_steering_module(std::make_shared<WrapSteering>(vp)));
    EXPECT_TRUE(l3->set_drivetrain_module(std::make_shared<WrapDrivetrain>(vp)));
    EXPECT_TRUE(l3->set_suspension_module(std::make_shared<WrapSuspension>(vp)));
    EXPECT_TRUE(l3->set_antirollbar_module(0, std::make_shared<WrapARB>(vp, 0)));
    EXPECT_TRUE(l3->set_antirollbar_module(1, std::make_shared<WrapARB>(vp, 1)));

    // L5 stunt: planar actuators only (no vertical suspension/ARB objects).
    auto l5 = vdsim::create_stunt_dof();
    l5->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
    EXPECT_TRUE(l5->set_brake_module(std::make_shared<WrapBrake>(vp)));
    EXPECT_TRUE(l5->set_steering_module(std::make_shared<WrapSteering>(vp)));
    EXPECT_TRUE(l5->set_drivetrain_module(std::make_shared<WrapDrivetrain>(vp)));
    EXPECT_FALSE(l5->set_suspension_module(std::make_shared<WrapSuspension>(vp)));

    // Rejections.
    EXPECT_FALSE(l2->set_brake_module(nullptr));
    EXPECT_FALSE(l3->set_antirollbar_module(2, std::make_shared<WrapARB>(vp, 1)));
}

// SimpleABS reference module: on a low-mu surface a full-brake stop locks the
// wheels with the default brake, but SimpleABS keeps them rolling (slip regulated).
TEST(UserModules, SimpleABSPreventsWheelLock) {
    vdsim::VehicleParams vp;
    const double mu = 0.2;   // icy

    auto run_min_wheel_ratio = [&](bool abs) {
        auto dyn = vdsim::create_seven_dof();
        dyn->initialize(vp, vdsim::TireParams{}, vdsim::SolverParams{});
        dyn->reset(rolling(20.0));
        if (abs) dyn->set_brake_module(std::make_shared<vdsim::SimpleABS>(vp));
        auto ground = vdsim::create_flat_ground(0.0, mu);
        vdsim::CmdL4 cmd; cmd.brake = 1.0;
        const vdsim::ControlInput u = cmd;
        double min_ratio = 1.0;
        for (int i = 0; i < 3000 && dyn->state().vx() > 1.0; ++i) {
            vdsim::ContactArray c; ground->query(dyn->state(), vp, c);
            dyn->step(u, c, 1e-3);
            const auto& s = dyn->state();
            const double ratio = s.wheel_spin[0] * vp.wheel_radius_nominal
                               / std::max(s.vx(), 0.5);
            min_ratio = std::min(min_ratio, ratio);
        }
        return min_ratio;
    };

    const double locked = run_min_wheel_ratio(false);  // wheels lock (ratio → 0)
    const double abs    = run_min_wheel_ratio(true);    // ABS keeps rolling
    EXPECT_LT(locked, 0.2)      << "default brake locks the wheel on ice";
    EXPECT_GT(abs,    0.5)      << "SimpleABS keeps the wheel rolling";
    EXPECT_GT(abs, locked + 0.3) << "ABS clearly reduces lock vs baseline";
}
