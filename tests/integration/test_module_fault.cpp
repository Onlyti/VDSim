// Work order 51b gates (0.9.0), fixed before the implementation (PO mail 16d10bd2).
//
// Every fault case runs in a child process (gtest EXPECT_EXIT, exit code 0 = pass), so a
// fault that still kills the process (std::terminate from a noexcept step, rc -6) fails
// that case instead of the whole test binary.
//
// Fault modules: one per user module kind, installed through IVehicleDynamics::set_*_module
// on every level whose setter returns true (L2, L3, L4, L5 penalty; a level that refuses a
// kind is printed and skipped). Before arming they return a benign output (brake /
// drivetrain zero torque, steering kinematic rack 0, suspension -30000 defl - 3000 rate,
// anti-roll bar (0, 0), aero no force). Session: generic sedan + generic tyre, flat ground
// mu 1, 1 ms ticks, start 10 m/s, CmdL4 throttle 0.1. The module is installed after
// reset_settled() (a reset puts the built-in brake / steering / drivetrain back). Ticks 1-4
// unarmed, the module is armed before tick 5.
//
// M2 module x fault: kinds brake, steering, drivetrain, suspension, antirollbar, aero x
//     {exception, NaN}. Exception: the module throws std::runtime_error("boom-<kind>#<n>"),
//     n = armed call count. NaN: one numeric output field is NaN; every field is a case:
//       brake wheel_torque[0..3]; steering rack_travel, motor_force, roadwheel_angle;
//       drivetrain wheel_torque[0..3], brake_absorbed; suspension force; antirollbar
//       force[0], force[1]; aero force_body.x/y/z, point.x/y/z.
//     Pass: tick 5 throws std::invalid_argument whose what() contains the kind name, the
//     module class name (Fault<Kind>), "tick 5" and the cause (exception text
//     "boom-<kind>#1", or the field name above). The process survives.
// M4a first fault only: the exception message carries "#1" and not "#2" (the armed module
//     throws again on later calls in the same tick).
// M4b after the fault, tick() again throws the same message (string equal).
// M4c output() after the fault is bit equal to output() after tick 4 (last good tick).
// M4d reset_settled() or restore(snapshot taken right after the first reset and install)
//     clears the fault: with a new disarmed module installed after it, 2000 ticks are bit
//     equal to a new session from the same start state with a new disarmed module.
// M4e VecSession, 4 sessions, 4 threads, only session 2 faulty (exception and NaN): the
//     exception reaches the caller of tick_all as std::invalid_argument naming the kind,
//     the process survives; the other sessions' vx is printed as information.
// M4 cases: brake module on L2 and L5 penalty.
// M3 (Python modules, VecSession threads): tests/scripts/test_module_fault.py.
// M1 (bytes / RL digests / closed-loop arrays / a benign module attached, against the base
//     tree), M5 (guard removed -> M2 fails), M6 (RL L2 preset A/B, 5-run median <= 3 %,
//     C++ brake module A/B as information) and M7 (canonical) are measurement procedures.
#include "vdsim/aero.hpp"
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/subsystems.hpp"
#include "vdsim/vec_session.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
constexpr double kDt = 1e-3;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

enum class Kind { Brake, Steering, Drivetrain, Suspension, AntiRollBar, Aero };
enum class Fault { Throw, NaN };

const char* kind_name(Kind k) {
    switch (k) {
        case Kind::Brake:       return "brake";
        case Kind::Steering:    return "steering";
        case Kind::Drivetrain:  return "drivetrain";
        case Kind::Suspension:  return "suspension";
        case Kind::AntiRollBar: return "antirollbar";
        case Kind::Aero:        return "aero";
    }
    return "?";
}
const char* class_name(Kind k) {
    switch (k) {
        case Kind::Brake:       return "FaultBrake";
        case Kind::Steering:    return "FaultSteering";
        case Kind::Drivetrain:  return "FaultDrivetrain";
        case Kind::Suspension:  return "FaultSuspension";
        case Kind::AntiRollBar: return "FaultArb";
        case Kind::Aero:        return "FaultAero";
    }
    return "?";
}
/// NaN field names per kind (index = field id).
std::vector<std::string> fields(Kind k) {
    switch (k) {
        case Kind::Brake:
            return {"wheel_torque[0]", "wheel_torque[1]", "wheel_torque[2]", "wheel_torque[3]"};
        case Kind::Steering:    return {"rack_travel", "motor_force", "roadwheel_angle"};
        case Kind::Drivetrain:
            return {"wheel_torque[0]", "wheel_torque[1]", "wheel_torque[2]", "wheel_torque[3]",
                    "brake_absorbed"};
        case Kind::Suspension:  return {"force"};
        case Kind::AntiRollBar: return {"force[0]", "force[1]"};
        case Kind::Aero:
            return {"force_body.x", "force_body.y", "force_body.z", "point.x", "point.y",
                    "point.z"};
    }
    return {};
}

struct Arm {
    bool armed {false};
    Fault fault {Fault::Throw};
    int field {0};
    int armed_calls {0};
    int calls {0};
    std::string kind;
    /// Count a call; throw when armed in exception mode. True = write the NaN field.
    bool hit() {
        ++calls;
        if (!armed) return false;
        ++armed_calls;
        if (fault == Fault::Throw)
            throw std::runtime_error("boom-" + kind + "#" + std::to_string(armed_calls));
        return true;
    }
};
using ArmPtr = std::shared_ptr<Arm>;

class FaultBrake : public IBrakeSystem {
public:
    explicit FaultBrake(ArmPtr a) : a_(std::move(a)) {}
    std::array<double, NUM_WHEELS> wheel_torque(const SubsystemContext&) override {
        std::array<double, NUM_WHEELS> t {{0.0, 0.0, 0.0, 0.0}};
        if (a_->hit()) t[a_->field] = kNaN;
        return t;
    }
private:
    ArmPtr a_;
};
class FaultSteering : public ISteeringSystem {
public:
    explicit FaultSteering(ArmPtr a) : a_(std::move(a)) {}
    SteeringOutput apply(const SubsystemContext&) override {
        SteeringOutput o;
        if (a_->hit()) {
            if (a_->field == 0) o.rack_travel = kNaN;
            if (a_->field == 1) o.motor_force = kNaN;
            if (a_->field == 2) o.roadwheel_angle = kNaN;
        }
        return o;
    }
private:
    ArmPtr a_;
};
class FaultDrivetrain : public IDrivetrain {
public:
    explicit FaultDrivetrain(ArmPtr a) : a_(std::move(a)) {}
    DrivetrainOutput apply(const SubsystemContext&) override {
        DrivetrainOutput o;
        if (a_->hit()) {
            if (a_->field < 4) o.wheel_torque[a_->field] = kNaN;
            else o.brake_absorbed = kNaN;
        }
        return o;
    }
private:
    ArmPtr a_;
};
class FaultSuspension : public ISuspension {
public:
    explicit FaultSuspension(ArmPtr a) : a_(std::move(a)) {}
    double force(const SubsystemContext&, const CornerInput& c) override {
        const double f = -30000.0 * c.defl - 3000.0 * c.defl_rate;
        return a_->hit() ? kNaN : f;
    }
private:
    ArmPtr a_;
};
class FaultArb : public IAntiRollBar {
public:
    explicit FaultArb(ArmPtr a) : a_(std::move(a)) {}
    std::pair<double, double> force(const SubsystemContext&, const AxleDefl&) override {
        std::pair<double, double> f {0.0, 0.0};
        if (a_->hit()) (a_->field == 0 ? f.first : f.second) = kNaN;
        return f;
    }
private:
    ArmPtr a_;
};
class FaultAero : public IAeroModel {
public:
    explicit FaultAero(ArmPtr a) : a_(std::move(a)) {}
    std::vector<AeroForce> compute(const SubsystemContext&, const AeroInput&) override {
        if (!a_->hit()) return {};
        AeroForce f;
        f.name = "probe";
        const int i = a_->field;
        if (i < 3) f.force_body[i] = kNaN;
        else f.point[i - 3] = kNaN;
        return {f};
    }
private:
    ArmPtr a_;
};

std::unique_ptr<SimSession> make_session(const std::string& level) {
    const VehicleParams vp = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    const TireParams tp = TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
    SolverParams sp;
    if (level == "L5") {
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
    }
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    return std::make_unique<SimSession>(create_dynamics_for_level(level), create_flat_ground(0.0, 1.0),
                                        vp, TireSetup(tp), sp, cfg);
}

State start_state(const SimSession& s) {
    (void)s;
    const VehicleParams vp = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity = Vec3(10.0, 0.0, 0.0);
    const double w = 10.0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    return s0;
}

/// Install a fault module of `k`; false when the level refuses that kind.
bool install(SimSession& s, Kind k, const ArmPtr& a) {
    auto& d = s.dynamics();
    switch (k) {
        case Kind::Brake:       return d.set_brake_module(std::make_shared<FaultBrake>(a));
        case Kind::Steering:    return d.set_steering_module(std::make_shared<FaultSteering>(a));
        case Kind::Drivetrain:  return d.set_drivetrain_module(std::make_shared<FaultDrivetrain>(a));
        case Kind::Suspension:  return d.set_suspension_module(std::make_shared<FaultSuspension>(a));
        case Kind::AntiRollBar: return d.set_antirollbar_module(0, std::make_shared<FaultArb>(a));
        case Kind::Aero:        return d.set_aero_module(std::make_shared<FaultAero>(a));
    }
    return false;
}

void drive_tick(SimSession& s) {
    CmdL4 u;
    u.throttle = 0.1;
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

bool bits_equal(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

bool has(const std::string& s, const std::string& t) { return s.find(t) != std::string::npos; }

/// Child body for one M2 case. Exit codes: 0 pass, 1 no exception, 2 not invalid_argument,
/// 3-6 message misses kind / class / tick / cause, 7 "#2" present, 8 install refused,
/// 9 module not called while armed.
int m2_case(const std::string& level, Kind k, Fault f, int field) {
    auto s = make_session(level);
    auto a = std::make_shared<Arm>();
    a->kind = kind_name(k);
    a->fault = f;
    a->field = field;
    s->reset_settled(start_state(*s));
    if (!install(*s, k, a)) return 8;  // after the reset: reset restores the built-in modules
    for (int t = 0; t < 4; ++t) drive_tick(*s);
    a->armed = true;
    std::string msg;
    int code = 0;
    try {
        drive_tick(*s);
        code = a->armed_calls == 0 ? 9 : 1;
    } catch (const std::invalid_argument& e) {
        msg = e.what();
    } catch (const std::exception& e) {
        msg = e.what();
        code = 2;
    }
    const std::string cause = f == Fault::Throw ? "boom-" + std::string(kind_name(k)) + "#1"
                                                : fields(k)[field];
    if (code == 0) {
        if (!has(msg, kind_name(k))) code = 3;
        else if (!has(msg, class_name(k))) code = 4;
        else if (!has(msg, "tick 5")) code = 5;
        else if (!has(msg, cause)) code = 6;
        else if (f == Fault::Throw && has(msg, "#2")) code = 7;
    }
    std::printf("M2 %s %-11s %s %-16s armed_calls=%d code=%d msg=\"%s\"\n", level.c_str(),
                kind_name(k), f == Fault::Throw ? "throw" : "nan  ",
                f == Fault::Throw ? "-" : fields(k)[field].c_str(), a->armed_calls, code,
                msg.c_str());
    std::fflush(stdout);
    return code;
}

/// True when `level` accepts `k` (checked in the parent; installing cannot fault).
bool accepts(const std::string& level, Kind k) {
    auto s = make_session(level);
    return install(*s, k, std::make_shared<Arm>());
}

const std::vector<std::string> kLevels {"L2", "L3", "L4", "L5"};
const std::vector<Kind> kKinds {Kind::Brake, Kind::Steering, Kind::Drivetrain, Kind::Suspension,
                                Kind::AntiRollBar, Kind::Aero};

}  // namespace

// ---- M2 / M4a -------------------------------------------------------------------------

TEST(ModuleFault, M2ExceptionBecomesInvalidArgumentAtTick) {
    for (Kind k : kKinds)
        for (const auto& lv : kLevels) {
            if (!accepts(lv, k)) {
                std::printf("M2 %s %s not accepted by this level (skipped)\n", lv.c_str(), kind_name(k));
                continue;
            }
            EXPECT_EXIT(std::_Exit(m2_case(lv, k, Fault::Throw, 0)), ::testing::ExitedWithCode(0), "")
                << lv << " " << kind_name(k);
        }
}

TEST(ModuleFault, M2NonFiniteOutputBecomesInvalidArgumentAtTick) {
    for (Kind k : kKinds)
        for (const auto& lv : kLevels) {
            if (!accepts(lv, k)) continue;
            for (int fi = 0; fi < (int)fields(k).size(); ++fi)
                EXPECT_EXIT(std::_Exit(m2_case(lv, k, Fault::NaN, fi)), ::testing::ExitedWithCode(0), "")
                    << lv << " " << kind_name(k) << " " << fields(k)[fi];
        }
}

// ---- M4b-d -------------------------------------------------------------------------------

namespace {
/// Exit codes: 0 pass; 1 no fault; 2 second tick message differs / no throw;
/// 3 output() not the last good output.
int m4bc_case(const std::string& level, Fault f) {
    auto s = make_session(level);
    auto a = std::make_shared<Arm>();
    a->kind = "brake";
    a->fault = f;
    a->field = 1;
    s->reset_settled(start_state(*s));
    install(*s, Kind::Brake, a);
    for (int t = 0; t < 4; ++t) drive_tick(*s);
    const auto good = out_bits(s->output());
    a->armed = true;
    std::string m1, m2;
    try { drive_tick(*s); } catch (const std::invalid_argument& e) { m1 = e.what(); }
    if (m1.empty()) return 1;
    const bool out_ok = bits_equal(out_bits(s->output()), good);
    try { drive_tick(*s); } catch (const std::invalid_argument& e) { m2 = e.what(); }
    std::printf("M4b/c %s %s first=\"%s\" again_equal=%d output_last_good=%d\n", level.c_str(),
                f == Fault::Throw ? "throw" : "nan", m1.c_str(), (int)(m2 == m1), (int)out_ok);
    std::fflush(stdout);
    if (m2 != m1) return 2;
    return out_ok ? 0 : 3;
}

std::vector<double> run_2000(SimSession& s) {
    std::vector<double> v;
    for (int t = 0; t < 2000; ++t) {
        drive_tick(s);
        const auto b = out_bits(s.output());
        v.insert(v.end(), b.begin(), b.end());
    }
    return v;
}

/// via_restore false = reset_settled; true = restore(snapshot after the first reset).
/// Exit codes: 0 pass; 1 no fault; 2 tick after clearing still throws; 3 trace differs.
int m4d_case(const std::string& level, bool via_restore) {
    auto s = make_session(level);
    auto a = std::make_shared<Arm>();
    a->kind = "brake";
    const State s0 = start_state(*s);
    s->reset_settled(s0);
    install(*s, Kind::Brake, a);
    const auto snap = s->snapshot();
    for (int t = 0; t < 4; ++t) drive_tick(*s);
    a->armed = true;
    bool faulted = false;
    try { drive_tick(*s); } catch (const std::invalid_argument&) { faulted = true; }
    if (!faulted) return 1;
    if (via_restore) s->restore(snap);
    else s->reset_settled(s0);
    install(*s, Kind::Brake, std::make_shared<Arm>());  // same disarmed module as the new session
    std::vector<double> got;
    try { got = run_2000(*s); } catch (const std::exception&) { return 2; }
    auto fresh = make_session(level);
    fresh->reset_settled(s0);
    install(*fresh, Kind::Brake, std::make_shared<Arm>());
    const auto want = run_2000(*fresh);
    const bool eq = bits_equal(got, want);
    std::printf("M4d %s %s values=%zu equal=%d\n", level.c_str(), via_restore ? "restore" : "reset",
                got.size(), (int)eq);
    std::fflush(stdout);
    return eq ? 0 : 3;
}

/// Exit codes: 0 pass; 1 no exception; 2 not invalid_argument; 3 message lacks "brake".
int m4e_case(Fault f) {
    std::vector<std::unique_ptr<SimSession>> v;
    auto a = std::make_shared<Arm>();
    a->kind = "brake";
    a->fault = f;
    a->field = 0;
    for (int i = 0; i < 4; ++i) {
        auto s = make_session("L2");
        s->reset_settled(start_state(*s));
        install(*s, Kind::Brake, i == 2 ? a : std::make_shared<Arm>());
        CmdL4 u;
        u.throttle = 0.1;
        s->set_input(u);
        v.push_back(std::move(s));
    }
    VecSession vs(std::move(v), 4);
    vs.tick_all(kDt, 4);
    a->armed = true;
    int code = 1;
    std::string msg;
    try {
        vs.tick_all(kDt, 1);
    } catch (const std::invalid_argument& e) {
        msg = e.what();
        code = has(msg, "brake") ? 0 : 3;
    } catch (const std::exception& e) {
        msg = e.what();
        code = 2;
    }
    std::printf("M4e %s code=%d msg=\"%s\" other vx=[%.6f %.6f %.6f] (info)\n",
                f == Fault::Throw ? "throw" : "nan", code, msg.c_str(),
                vs.at(0).output().state.velocity.x(), vs.at(1).output().state.velocity.x(),
                vs.at(3).output().state.velocity.x());
    std::fflush(stdout);
    return code;
}
}  // namespace

TEST(ModuleFault, M4bcRepeatTickSameMessageAndLastGoodOutput) {
    for (const char* lv : {"L2", "L5"})
        for (Fault f : {Fault::Throw, Fault::NaN})
            EXPECT_EXIT(std::_Exit(m4bc_case(lv, f)), ::testing::ExitedWithCode(0), "") << lv;
}

TEST(ModuleFault, M4dResetOrRestoreClearsFault) {
    for (const char* lv : {"L2", "L5"})
        for (bool r : {false, true})
            EXPECT_EXIT(std::_Exit(m4d_case(lv, r)), ::testing::ExitedWithCode(0), "") << lv << r;
}

TEST(ModuleFault, M4eVecSessionFaultReachesCaller) {
    for (Fault f : {Fault::Throw, Fault::NaN})
        EXPECT_EXIT(std::_Exit(m4e_case(f)), ::testing::ExitedWithCode(0), "");
}
