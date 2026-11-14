// Work order 51c (PROP-2794 option 1): reset keeps user modules. Gates fixed before the
// implementation; the header text is the gate definition.
//
// Session: generic sedan + generic tyre, 1 ms ticks, start 10 m/s straight; L5 = stunt
// physics penalty contact (same as 51b). Drive input per tick index k (restarts at 0 after
// every reset / restore): CmdL4 throttle 0.25 for k mod 600 < 250, brake 0.3 for
// 300 <= k mod 600 < 450, road-wheel steer 0.03 sin(2 pi k / 700) rad.
// Test modules ("keep" modules): each wraps a built-in default module of the same kind
// (make_default_*, built from the same vehicle with the same dead time), scales its output
// (brake torque x0.7, steering rack travel x0.8, drivetrain wheel torque x0.9, suspension
// force x1.1, anti-roll bar forces x1.2) and counts its force calls; reset() counts and
// rebuilds the wrapped built-in module, so a reset module equals a new one.
// Cases: L2, L3, L4, L5 x brake, steering, drivetrain; L3, L4 x suspension, anti-roll bar
// (one module per axle, both counted together).
//
// R1 attach after reset_settled(s0) -> 200 ticks -> reset_settled(s0) -> 200 ticks: force
//    calls after the second reset > 0 and module reset() calls >= 1 since the first 200
//    ticks (the count itself is not judged). Plus one randomized-reset case (L2 brake,
//    set_randomization mass_scale 1.05 before the second reset): same judgement.
// R2 every R1 case: the 2000 ticks after the second reset are bit equal to a new session
//    (reset_settled(s0), a new keep module attached, 2000 ticks) on the 51b output vector.
// R4 attach after reset_settled(s0) -> snapshot -> 200 ticks -> restore(snapshot) -> 200
//    ticks: same judgement as R1, every R1 case.
// R7 L2 and L5, brake: a module that throws when armed faults the session (tick throws
//    std::invalid_argument); reset_settled(s0) clears the fault; disarmed, 10 ticks raise
//    nothing and call the same module again; armed again, the next tick throws
//    std::invalid_argument naming "brake" and the class "ArmBrake" (guard still in place).
//    A module whose reset() throws: reset_settled(s0) returns (process alive) and the next
//    tick throws std::invalid_argument whose text has "brake" and "reset".
// R3 (Python modules, VecSession): tests/scripts/test_module_reset_keep.py.
// R5 (unattached paths: bytes 24, RL digests 2, closed loop 505 deterministic keys against
//    ab9f5a6), R6 (51b M1 built-in-as-user 24 cases), N1 (reverting the 51c change makes
//    R1 and R2 fail, restoring passes) and R8 (canonical, name diff) are procedures.
#include "vdsim/control.hpp"
#include "vdsim/default_subsystems.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/subsystems.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
constexpr double kDt = 1e-3;
constexpr double kPi = 3.14159265358979323846;

enum class Kind { Brake, Steering, Drivetrain, Suspension, AntiRollBar };

const char* kind_name(Kind k) {
    switch (k) {
        case Kind::Brake:       return "brake";
        case Kind::Steering:    return "steering";
        case Kind::Drivetrain:  return "drivetrain";
        case Kind::Suspension:  return "suspension";
        case Kind::AntiRollBar: return "antirollbar";
    }
    return "?";
}

VehicleParams sedan() {
    return VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
}

std::unique_ptr<SimSession> make_session(const std::string& level) {
    const TireParams tp = TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
    SolverParams sp;
    if (level == "L5") {
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
    }
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    return std::make_unique<SimSession>(create_dynamics_for_level(level),
                                        create_flat_ground(0.0, 1.0), sedan(), TireSetup(tp),
                                        sp, cfg);
}

State start_state() {
    const VehicleParams vp = sedan();
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity = Vec3(10.0, 0.0, 0.0);
    const double w = 10.0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    return s0;
}

struct Count {
    int calls {0};
    int resets {0};
};
using CountPtr = std::shared_ptr<Count>;

class KeepBrake final : public IBrakeSystem {
public:
    KeepBrake(const VehicleParams& vp, CountPtr c) : vp_(vp), c_(std::move(c)) { rebuild(); }
    LcLevel min_lon_level() const override { return inner_->min_lon_level(); }
    LcLevel max_lon_level() const override { return inner_->max_lon_level(); }
    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext& ctx) override {
        ++c_->calls;
        auto t = inner_->wheel_torque(ctx);
        for (double& x : t) x *= 0.7;
        return t;
    }
    void begin_step(const SubsystemContext& ctx, double dt) override { inner_->begin_step(ctx, dt); }
    void reset() override {
        ++c_->resets;
        rebuild();
    }

private:
    void rebuild() { inner_ = make_default_brake(vp_, vp_.brake_deadtime_s); }
    VehicleParams vp_;
    CountPtr c_;
    std::unique_ptr<IBrakeSystem> inner_;
};

class KeepSteering final : public ISteeringSystem {
public:
    KeepSteering(const VehicleParams& vp, CountPtr c) : vp_(vp), c_(std::move(c)) { rebuild(); }
    LcLevel min_lat_level() const override { return inner_->min_lat_level(); }
    LcLevel max_lat_level() const override { return inner_->max_lat_level(); }
    SteeringOutput apply(const SubsystemContext& ctx) override {
        ++c_->calls;
        SteeringOutput o = inner_->apply(ctx);
        o.rack_travel *= 0.8;
        return o;
    }
    void begin_step(const SubsystemContext& ctx, double dt) override { inner_->begin_step(ctx, dt); }
    void reset() override {
        ++c_->resets;
        rebuild();
    }

private:
    void rebuild() { inner_ = make_default_steering(vp_, vp_.steer_deadtime_s); }
    VehicleParams vp_;
    CountPtr c_;
    std::unique_ptr<ISteeringSystem> inner_;
};

class KeepDrivetrain final : public IDrivetrain {
public:
    KeepDrivetrain(const VehicleParams& vp, CountPtr c) : vp_(vp), c_(std::move(c)) { rebuild(); }
    LcLevel min_lon_level() const override { return inner_->min_lon_level(); }
    LcLevel max_lon_level() const override { return inner_->max_lon_level(); }
    DrivetrainOutput apply(const SubsystemContext& ctx) override {
        ++c_->calls;
        DrivetrainOutput o = inner_->apply(ctx);
        for (double& x : o.wheel_torque) x *= 0.9;
        return o;
    }
    void begin_step(const SubsystemContext& ctx, double dt) override { inner_->begin_step(ctx, dt); }
    void reset() override {
        ++c_->resets;
        rebuild();
    }
    double engine_rpm() const override { return inner_->engine_rpm(); }
    int current_gear() const override { return inner_->current_gear(); }
    double wheel_engine_inertia(int w) const override { return inner_->wheel_engine_inertia(w); }

private:
    void rebuild() { inner_ = make_default_drivetrain(vp_, vp_.drive_deadtime_s); }
    VehicleParams vp_;
    CountPtr c_;
    std::unique_ptr<IDrivetrain> inner_;
};

class KeepSuspension final : public ISuspension {
public:
    KeepSuspension(const VehicleParams& vp, CountPtr c) : vp_(vp), c_(std::move(c)) { rebuild(); }
    double force(const SubsystemContext& ctx, const CornerInput& ci) override {
        ++c_->calls;
        return 1.1 * inner_->force(ctx, ci);
    }
    void reset() override {
        ++c_->resets;
        rebuild();
    }

private:
    void rebuild() { inner_ = make_default_suspension(vp_); }
    VehicleParams vp_;
    CountPtr c_;
    std::unique_ptr<ISuspension> inner_;
};

class KeepArb final : public IAntiRollBar {
public:
    KeepArb(const VehicleParams& vp, int axle, CountPtr c)
        : vp_(vp), axle_(axle), c_(std::move(c)) { rebuild(); }
    std::pair<double, double> force(const SubsystemContext& ctx, const AxleDefl& d) override {
        ++c_->calls;
        auto f = inner_->force(ctx, d);
        f.first *= 1.2;
        f.second *= 1.2;
        return f;
    }
    void reset() override {
        ++c_->resets;
        rebuild();
    }

private:
    void rebuild() { inner_ = make_default_antirollbar(vp_, axle_); }
    VehicleParams vp_;
    int axle_;
    CountPtr c_;
    std::unique_ptr<IAntiRollBar> inner_;
};

bool attach(SimSession& s, Kind k, const CountPtr& c) {
    const VehicleParams vp = sedan();
    auto& d = s.dynamics();
    switch (k) {
        case Kind::Brake:      return d.set_brake_module(std::make_shared<KeepBrake>(vp, c));
        case Kind::Steering:   return d.set_steering_module(std::make_shared<KeepSteering>(vp, c));
        case Kind::Drivetrain: return d.set_drivetrain_module(std::make_shared<KeepDrivetrain>(vp, c));
        case Kind::Suspension: return d.set_suspension_module(std::make_shared<KeepSuspension>(vp, c));
        case Kind::AntiRollBar:
            return d.set_antirollbar_module(0, std::make_shared<KeepArb>(vp, 0, c)) &&
                   d.set_antirollbar_module(1, std::make_shared<KeepArb>(vp, 1, c));
    }
    return false;
}

struct Case {
    std::string level;
    Kind kind;
};

std::vector<Case> cases() {
    std::vector<Case> v;
    for (const char* l : {"L2", "L3", "L4", "L5"})
        for (Kind k : {Kind::Brake, Kind::Steering, Kind::Drivetrain}) v.push_back({l, k});
    for (const char* l : {"L3", "L4"})
        for (Kind k : {Kind::Suspension, Kind::AntiRollBar}) v.push_back({l, k});
    return v;
}

void drive_tick(SimSession& s, int k) {
    CmdL4 u;
    const int ph = k % 600;
    u.throttle = ph < 250 ? 0.25 : 0.0;
    u.brake = (ph >= 300 && ph < 450) ? 0.3 : 0.0;
    u.steer_angle_wheel = 0.03 * std::sin(2.0 * kPi * k / 700.0);
    s.set_input(u);
    s.tick(kDt);
}

std::vector<double> out_bits(const SimOutput& o) {
    std::vector<double> v;
    const auto& st = o.state;
    for (double x : {o.sim_time, st.position.x(), st.position.y(), st.position.z(),
                     st.velocity.x(), st.velocity.y(), st.velocity.z(), st.angular_velocity.x(),
                     st.angular_velocity.y(), st.angular_velocity.z(), st.orientation.w(),
                     st.orientation.x(), st.orientation.y(), st.orientation.z(), o.ax, o.ay,
                     o.brake_applied})
        v.push_back(x);
    for (int i = 0; i < NUM_WHEELS; ++i) {
        v.push_back(st.wheel_spin[i]);
        v.push_back(o.Fz[i]);
        for (int j = 0; j < 3; ++j) {
            v.push_back(o.tire_forces_wheel[i][j]);
            v.push_back(o.tire_forces[i][j]);
        }
    }
    return v;
}

std::vector<double> run_record(SimSession& s, int n) {
    std::vector<double> v;
    for (int k = 0; k < n; ++k) {
        drive_tick(s, k);
        const auto b = out_bits(s.output());
        v.insert(v.end(), b.begin(), b.end());
    }
    return v;
}

bool bits_equal(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

bool has(const std::string& s, const std::string& t) { return s.find(t) != std::string::npos; }

}  // namespace

TEST(ModuleResetKeep, R1UserModuleCalledAfterReset) {
    const State s0 = start_state();
    for (const auto& c : cases()) {
        auto s = make_session(c.level);
        s->reset_settled(s0);
        auto cnt = std::make_shared<Count>();
        ASSERT_TRUE(attach(*s, c.kind, cnt)) << c.level << " " << kind_name(c.kind);
        for (int k = 0; k < 200; ++k) drive_tick(*s, k);
        const int calls_before = cnt->calls, resets_before = cnt->resets;
        s->reset_settled(s0);
        const int calls_at_reset = cnt->calls;
        for (int k = 0; k < 200; ++k) drive_tick(*s, k);
        const int calls_after = cnt->calls - calls_at_reset;
        const int resets = cnt->resets - resets_before;
        std::printf("R1 %s %-11s calls_before=%d calls_after=%d resets=%d\n", c.level.c_str(),
                    kind_name(c.kind), calls_before, calls_after, resets);
        EXPECT_GT(calls_before, 0) << c.level << " " << kind_name(c.kind);
        EXPECT_GT(calls_after, 0) << c.level << " " << kind_name(c.kind);
        EXPECT_GE(resets, 1) << c.level << " " << kind_name(c.kind);
    }
    {
        auto s = make_session("L2");
        s->reset_settled(s0);
        auto cnt = std::make_shared<Count>();
        ASSERT_TRUE(attach(*s, Kind::Brake, cnt));
        for (int k = 0; k < 200; ++k) drive_tick(*s, k);
        const int resets_before = cnt->resets;
        DomainRandomization d;
        d.mass_scale = 1.05;
        s->set_randomization(d);
        s->reset_settled(s0);
        const int calls_at_reset = cnt->calls;
        for (int k = 0; k < 200; ++k) drive_tick(*s, k);
        std::printf("R1 L2 brake randomized calls_after=%d resets=%d\n",
                    cnt->calls - calls_at_reset, cnt->resets - resets_before);
        EXPECT_GT(cnt->calls - calls_at_reset, 0) << "randomized reset";
        EXPECT_GE(cnt->resets - resets_before, 1) << "randomized reset";
    }
}

TEST(ModuleResetKeep, R2ResetEqualsNewSessionWithSameModule) {
    const State s0 = start_state();
    for (const auto& c : cases()) {
        auto a = make_session(c.level);
        a->reset_settled(s0);
        ASSERT_TRUE(attach(*a, c.kind, std::make_shared<Count>()));
        for (int k = 0; k < 200; ++k) drive_tick(*a, k);
        a->reset_settled(s0);
        const auto got = run_record(*a, 2000);

        auto b = make_session(c.level);
        b->reset_settled(s0);
        ASSERT_TRUE(attach(*b, c.kind, std::make_shared<Count>()));
        const auto want = run_record(*b, 2000);
        const bool eq = bits_equal(got, want);
        std::printf("R2 %s %-11s values=%zu equal=%d\n", c.level.c_str(), kind_name(c.kind),
                    got.size(), (int)eq);
        EXPECT_TRUE(eq) << c.level << " " << kind_name(c.kind);
    }
}

TEST(ModuleResetKeep, R4UserModuleCalledAfterRestore) {
    const State s0 = start_state();
    for (const auto& c : cases()) {
        auto s = make_session(c.level);
        s->reset_settled(s0);
        auto cnt = std::make_shared<Count>();
        ASSERT_TRUE(attach(*s, c.kind, cnt));
        const auto snap = s->snapshot();
        for (int k = 0; k < 200; ++k) drive_tick(*s, k);
        const int resets_before = cnt->resets;
        s->restore(snap);
        const int calls_at_restore = cnt->calls;
        for (int k = 0; k < 200; ++k) drive_tick(*s, k);
        const int calls_after = cnt->calls - calls_at_restore;
        const int resets = cnt->resets - resets_before;
        std::printf("R4 %s %-11s calls_after=%d resets=%d\n", c.level.c_str(),
                    kind_name(c.kind), calls_after, resets);
        EXPECT_GT(calls_after, 0) << c.level << " " << kind_name(c.kind);
        EXPECT_GE(resets, 1) << c.level << " " << kind_name(c.kind);
    }
}

namespace {
struct ArmState {
    bool armed {false};
    bool throw_on_reset {false};
    int calls {0};
};
using ArmStatePtr = std::shared_ptr<ArmState>;

class ArmBrake final : public IBrakeSystem {
public:
    explicit ArmBrake(ArmStatePtr a) : a_(std::move(a)) {}
    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext&) override {
        ++a_->calls;
        if (a_->armed) throw std::runtime_error("boom-brake");
        return {{0.0, 0.0, 0.0, 0.0}};
    }
    void reset() override {
        if (a_->throw_on_reset) throw std::runtime_error("boom-reset");
    }

private:
    ArmStatePtr a_;
};

std::string tick_error(SimSession& s, int k) {
    try {
        drive_tick(s, k);
    } catch (const std::invalid_argument& e) {
        return std::string("IA:") + e.what();
    } catch (const std::exception& e) {
        return std::string("OTHER:") + e.what();
    }
    return {};
}
}  // namespace

TEST(ModuleResetKeep, R7FaultThenResetKeepsGuardedModule) {
    const State s0 = start_state();
    for (const char* level : {"L2", "L5"}) {
        auto s = make_session(level);
        s->reset_settled(s0);
        auto a = std::make_shared<ArmState>();
        ASSERT_TRUE(s->dynamics().set_brake_module(std::make_shared<ArmBrake>(a)));
        for (int k = 0; k < 4; ++k) drive_tick(*s, k);
        a->armed = true;
        const std::string e1 = tick_error(*s, 4);
        s->reset_settled(s0);
        a->armed = false;
        const int c0 = a->calls;
        std::string e2;
        for (int k = 0; k < 10 && e2.empty(); ++k) e2 = tick_error(*s, k);
        const int calls_after = a->calls - c0;
        a->armed = true;
        const std::string e3 = tick_error(*s, 10);
        std::printf("R7 %s first=\"%s\" after_reset_err=\"%s\" calls_after=%d again=\"%s\"\n",
                    level, e1.c_str(), e2.c_str(), calls_after, e3.c_str());
        EXPECT_EQ(e1.rfind("IA:", 0), 0u) << level;
        EXPECT_TRUE(e2.empty()) << level;
        EXPECT_GT(calls_after, 0) << level;
        EXPECT_EQ(e3.rfind("IA:", 0), 0u) << level;
        EXPECT_TRUE(has(e3, "brake") && has(e3, "ArmBrake")) << level << " " << e3;
    }
}

TEST(ModuleResetKeep, R7ResetThatThrowsFaultsTheNextTick) {
    const State s0 = start_state();
    for (const char* level : {"L2", "L5"}) {
        auto s = make_session(level);
        s->reset_settled(s0);
        auto a = std::make_shared<ArmState>();
        ASSERT_TRUE(s->dynamics().set_brake_module(std::make_shared<ArmBrake>(a)));
        for (int k = 0; k < 4; ++k) drive_tick(*s, k);
        a->throw_on_reset = true;
        s->reset_settled(s0);  // the process must survive the module's reset() throwing
        const std::string e = tick_error(*s, 0);
        std::printf("R7r %s next_tick=\"%s\"\n", level, e.c_str());
        EXPECT_EQ(e.rfind("IA:", 0), 0u) << level;
        EXPECT_TRUE(has(e, "brake") && has(e, "reset")) << level << " " << e;
    }
}
