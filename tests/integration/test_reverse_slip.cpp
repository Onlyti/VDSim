// Work order 57 gates (0.8.3), fixed before the implementation commits.
//
// U2  reverse small slip, tyre level. kappa = 0, static load Fz = m g / 4 (generic sedan),
//     |vx| = 10 m/s, alpha = 1 deg and 5 deg (vy = 10 tan(alpha) > 0):
//       - contact slip angle when reversing = atan(vy / |vx|) within 1e-12 rad
//         (tire_contact_kinematics and the model's reported alpha),
//       - steady Fy(reverse) = Fy(forward, same vy) within +-0.1 %,
//       - Fy(reverse) * vy < 0.
//     Tyres: configs/parts/tire/generic_pacejka.yaml as shipped (LuGre on) and the same
//     file with LuGre off (pure magic formula). Steady state = 2 s of bristle advance at
//     1e-4 s (LuGre), kappa = 0 by omega = vx / Re.
// U3  reverse acceleration command (CmdL5 ax_target), vx0 = -3 m/s, generic sedan, flat
//     ground mu 1, 1 ms ticks, 2 s, L2 and L5 penalty:
//       (a) ax_target = +1 (reverse deceleration): mean ax over ticks with t in [0.5, 1.5) s
//           = +1 +-10 %, and |vx(1.5 s)| < |vx(0.5 s)|.
//       (b) ax_target = -0.5 (reverse acceleration): brake_applied == 0 on every tick,
//           mean ax over [0.5, 1.5) s < 0.
//     Gate change (user decision PROP-2783, after the measurement, PO mail c337f6ec): the
//     U3a absolute criterion (mean ax = +1 +-10 %) is printed as information and no longer
//     judged; U3a-sym below is the U3a gate. U3a keeps its finiteness and |vx| checks.
// U3a-sym (added 11-14 by PO mail 13399c6a before the B commit; the U3a gate since
//     PROP-2783):
//     same |vx0| = 3 m/s, |a_cmd| = 1 m/s^2, same window [0.5, 1.5) s, L2 and L5 penalty.
//     Reverse deceleration (vx0 = -3, ax_target = +1) against forward deceleration
//     (vx0 = +3, ax_target = -1):
//       |mean_ax(rev) - (-mean_ax(fwd))| <= 0.02 |mean_ax(fwd)|,
//     and the reverse run brakes (brake_applied > 0 on some tick) and |vx| falls
//     (|vx(1.5 s)| < |vx(0.5 s)|).
// U3c (PROP-2785 (b), work order 57 section 7.2): RL direct-control pedal path, Python side,
//     tests/scripts/test_reverse_rl_pedal.py (ctest reverse_rl_pedal).
// U3d (PROP-2785 (d)): DriverModel speed tracking while reversing. Generic sedan, flat
//     ground mu 1, 1 ms ticks, 2 s, vx0 = -3 m/s, L2 and L5 penalty, DriverModel noise 0,
//     steer to the plant 0 (the longitudinal chain is judged), the pedals and the gear
//     the model returns go to CmdL4 (gear +1 while the model reports none):
//       (a) reverse acceleration request v_target = -5 m/s: brake_applied == 0 on every
//           tick and mean ax over t in [0.5, 1.5) s < 0,
//       (b) reverse deceleration request v_target = -1 m/s: |vx(1.5 s)| < |vx(0.5 s)|,
//       every value finite.
//     Forward (vx0 = +3, v_target = +5 and +1): every tick, throttle and brake bit equal
//     to the pre-57 chain LongVxController::update -> LongAxController::update with the
//     same gains, and the gear is +1 (U1 for the driver model).
// K1 (PROP-2784 option 2, section 7.3): relaxation effect, tyre alone. Generic tyre with
//     the relaxed coefficients of vdsim/tire_low_speed.hpp (belt on, sigma_y =
//     kRelaxedSigmaLat, sigma_x = kRelaxedSigmaLong), Fz = m g / 4, kappa = 0 by omega =
//     vx / Re before the step. |vx| = 5 and 10 m/s, forward and reverse; step at t = 0
//     of alpha 0 -> 1 deg (vy = |vx| tan 1 deg, sigma_y) and of kappa 0 -> 0.02 (omega =
//     (vx + 0.02 |vx|) / Re, sigma_x). Judged quantity = the relaxed slip the force law
//     reads (MF path, LuGre off: Transient belt_alpha / belt_kappa; LuGre path: belt_vlat
//     / belt_vlong), advanced by advance_relaxation at dt = tau / 200 (record interval
//     <= tau / 100). The first time it reaches (1 - 1/e) of its step (linear interpolation
//     between records) = tau = sigma / |vx| within +-5 %. The force crossing time is
//     printed as information (the MF force law is mildly non-linear in the slip, and the
//     LuGre bristle adds its own lag). All judged speeds are above kRelaxedVLow.
// K2 low-speed regression, L2 and L5 penalty, key `relaxed` against the key unset (off):
//     C1 CmdL5 ax_target -1 from 5 m/s, switch to ax_target 0 at the first |vx| < 0.1 m/s,
//        then 3 s; C3 state F (vx 0, vy 3, r 0), CmdL5 0, 3 s.
//     Last 1 s: per wheel the number of Fx sign changes (consecutive ticks, exact zeros
//     skipped) with relaxed <= the same count with off; every value finite; |vx| < 0.01
//     m/s on the last tick (relaxed). C3 Fy sign changes are printed as information.
//     The U5 brake-hold run below stays as a regression item (relaxed keeps standstill).
// K3 DAE oscillation non-worsening: measured after 49a is on main, with the 49a P1 test
//     (antidive and sports DAE, 25 and 35 m/s, command 0, last 5 s q_rms <= 1e-3 rad/s,
//     growth < 0.5) run with tire_low_speed: relaxed. Its test is added in a separate gate
//     commit after the main merge, before that measurement; sigma is not tuned on a miss.
// K4 key contract: the U5 off == unset bit test and the ReverseSlipKeys tests below, and
//     the Python test tire_low_speed.
// K4b the key acts on the session (K1 checks the filter alone): L2 and L5 penalty, 10 m/s,
//     CmdL4 throttle 0 brake 0, road-wheel steer 0 -> 0.05 rad at t = 0.1 s; mean |ay| over
//     ticks with t in (0.1, 0.15] s with relaxed < the same mean with the key unset (the
//     relaxed tyre builds lateral force later), every value finite.
// K5 negative check (procedure, not a test): removing only the relaxation filter makes
//     K1 fail; restoring it passes.
// U4  pure side slide / reverse spin: the 11-13 probe states (vx < 0, vx ~ 0 with large
//     |vy|, vx < 0 with large r) on the generic sedan, CmdL5 ax_target 0, steer 0, 2 s:
//       - every state / output value finite (no NaN, no Inf) on every tick,
//       - per wheel |Fy| <= mu Fz on every tick, mu = tyre mu_nominal x ground mu (1.0),
//         Fy = tire_forces_wheel[i].y(), Fz = output Fz[i] (relative 1e-9 + 1e-6 N slack
//         for rounding only).
//     Paths: L2 kinematic blend, L2 plant path (VehicleParams::plant_path, the
//     VDSimPlant setting of the probe), L5 penalty.
//     Measurement correction (11-14, PO mail 13399c6a rule (ga), limit unchanged): the
//     L5 free-3D model reports tire_forces_wheel in the body frame, so its y component
//     carries Fz x (body roll) and is not the tyre lateral force. For L5 penalty the
//     judged force is the contact-plane magnitude |F - (F.n_b) n_b|, n_b = R^T e_z (flat
//     ground normal in the body frame), which bounds the tyre lateral force from above.
//     Diagnostic before this change: every RK stage had |Fy_tyre| <= mu Fz_in.
// U5  (replaced by K1-K5 under PROP-2784; the brake-hold run stays as a regression item)
//     low-speed key `tire_low_speed: relaxed` (vehicle YAML): start 5 m/s, constant brake
//     pedal 0.3 (CmdL4), t_stop = first tick with |vx| < 0.01 m/s (must come within 6 s),
//     then 3 s more:
//       - hold: |vx| <= 0.05 m/s on every tick of (t_stop, t_stop + 3],
//       - last 1 s (t_stop + 2, t_stop + 3]: per wheel the sign of Fx =
//         tire_forces_wheel[i].x() changes at most 2 times (consecutive ticks; ticks with
//         Fx exactly 0 are skipped),
//       - every value finite over the whole run. L2 and L5 penalty.
//     `off` written explicitly = key unset, state bits identical over the same run.
//     DAE fixture P1 non-worsening is measured with the 49a P1 test after 49a reaches
//     main (the P1 test is not on this base).
// Key contract (C++): off | relaxed YAML -> to_yaml round trip, unset writes no key,
//     unknown value -> std::invalid_argument from VehicleParams::from_yaml.
//     Checked on the YAML text so this file builds before the key exists.
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/control_converter.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/tire_contact.hpp"
#include "vdsim/tire_low_speed.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
const std::string kSedan = kSrc + "/configs/vehicles/generic_sedan.yaml";
const std::string kTire = kSrc + "/configs/parts/tire/generic_pacejka.yaml";
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1e-3;

std::string read_all(const std::string& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string write_tmp(const std::string& name, const std::string& text) {
    const std::string p = ::testing::TempDir() + name;
    std::ofstream(p) << text;
    return p;
}

/// Generic sedan, optionally with `tire_low_speed: <v>` appended to the YAML.
VehicleParams sedan(const std::string& low_speed = "") {
    if (low_speed.empty()) return VehicleParams::from_yaml(kSedan);
    const auto p = write_tmp("u57_sedan_" + low_speed + ".yaml",
                             read_all(kSedan) + "\ntire_low_speed: " + low_speed + "\n");
    return VehicleParams::from_yaml(p);
}

TireParams tire(bool lugre = true) {
    TireParams tp = TireParams::from_yaml(kTire);
    if (!lugre) tp.lugre.enabled = false;
    return tp;
}

/// Steady tyre wrench at kappa = 0 (same procedure as the 11-14 pre-fix probe).
ITireModel::Wrench steady(const ITireModel& t, double vx, double vy, double Fz, double R0) {
    ITireModel::ContactInput ci;
    ci.Fz = Fz;
    ci.Vx = vx;
    ci.Vy = vy;
    ci.R0 = R0;
    ITireModel::Transient tr;
    ci.omega = vx / R0;
    auto w = t.evaluate(ci, tr);
    ci.omega = vx / w.Re;
    for (int k = 0; k < 20000; ++k) tr = t.advance_bristle(ci, tr, 1e-4);
    return t.evaluate(ci, tr);
}

enum class Path { L2, L2Plant, L5Penalty };

const char* path_name(Path p) {
    switch (p) {
        case Path::L2:        return "L2";
        case Path::L2Plant:   return "L2plant";
        case Path::L5Penalty: return "L5pen";
    }
    return "?";
}

std::unique_ptr<SimSession> session(Path p, VehicleParams vp, const TireParams& tp) {
    SolverParams sp;
    if (p == Path::L5Penalty) {
        sp.stunt_physics = true;
        sp.max_substep_dt = 2e-4;
        sp.max_substeps = 16;
    }
    if (p == Path::L2Plant) vp.plant_path = true;
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    return std::make_unique<SimSession>(
        create_dynamics_for_level(p == Path::L5Penalty ? "L5" : "L2"),
        create_flat_ground(0.0, 1.0), vp, TireSetup(tp), sp, cfg);
}

/// Body-frame start state with free-rolling wheels (spin = vx / R).
State start(const VehicleParams& vp, double vx, double vy, double r) {
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity = Vec3(vx, vy, 0.0);
    s0.angular_velocity = Vec3(0.0, 0.0, r);
    const double w = vx / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    return s0;
}

bool finite3(const Vec3& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool all_finite(const SimOutput& o) {
    const auto& s = o.state;
    if (!finite3(s.position) || !finite3(s.velocity) || !finite3(s.angular_velocity)) return false;
    if (!std::isfinite(o.ax) || !std::isfinite(o.ay)) return false;
    for (int i = 0; i < NUM_WHEELS; ++i) {
        if (!std::isfinite(s.wheel_spin[i]) || !std::isfinite(o.Fz[i])) return false;
        if (!finite3(o.tire_forces_wheel[i]) || !finite3(o.tire_forces[i])) return false;
    }
    return true;
}

}  // namespace

// ---- U2 -----------------------------------------------------------------------------

TEST(ReverseSlip, U2ContactSlipAngleUsesAbsVx) {
    const TireParams tp = tire(true);
    const double R0 = sedan().wheel_radius_nominal;
    for (double adeg : {1.0, 5.0}) {
        const double vy = 10.0 * std::tan(adeg * kPi / 180.0);
        const double want = std::atan(vy / 10.0);
        const auto ck = tire_contact_kinematics(-10.0, vy, -10.0 / R0, 4000.0, 0.0, tp, R0);
        std::printf("U2 kin alpha=%gdeg rev alpha=%.15f want=%.15f err=%.3e\n", adeg, ck.alpha,
                    want, ck.alpha - want);
        EXPECT_NEAR(ck.alpha, want, 1e-12) << "alpha " << adeg << " deg";
    }
}

TEST(ReverseSlip, U2ReverseLateralForceEqualsForward) {
    const VehicleParams vp = sedan();
    const double Fz = vp.mass * 9.81 / 4.0;
    const double R0 = vp.wheel_radius_nominal;
    for (bool lugre : {true, false}) {
        auto t = make_tire_model(tire(lugre));
        for (double adeg : {1.0, 5.0}) {
            const double vy = 10.0 * std::tan(adeg * kPi / 180.0);
            const double want = std::atan(vy / 10.0);
            const auto f = steady(*t, 10.0, vy, Fz, R0);
            const auto r = steady(*t, -10.0, vy, Fz, R0);
            const double rel = r.Fy / f.Fy - 1.0;
            std::printf("U2 lugre=%d a=%gdeg fwd Fy=%.6f rev Fy=%.6f rel=%+.6f %% rev alpha=%.15f\n",
                        (int)lugre, adeg, f.Fy, r.Fy, 100.0 * rel, r.alpha);
            EXPECT_NEAR(r.alpha, want, 1e-12) << "lugre " << lugre << " " << adeg << " deg";
            EXPECT_LE(std::abs(rel), 1e-3) << "lugre " << lugre << " " << adeg << " deg";
            EXPECT_LT(r.Fy * vy, 0.0) << "lugre " << lugre << " " << adeg << " deg";
        }
    }
}

// ---- U3 -----------------------------------------------------------------------------

namespace {
struct U3Run {
    double mean_ax {0.0}, vx05 {0.0}, vx15 {0.0}, brake_max {0.0};
    bool finite {true};
};

U3Run run_u3(Path p, double a_cmd, double vx0 = -3.0) {
    const VehicleParams vp = sedan();
    auto s = session(p, vp, tire(true));
    s->reset_settled(start(vp, vx0, 0.0, 0.0));
    U3Run out;
    double sum = 0.0;
    int n = 0;
    for (int k = 0; k < 2000; ++k) {
        CmdL5 u;
        u.ax_target = a_cmd;
        s->set_input(u);
        s->tick(kDt);
        const auto o = s->output();
        const double t = (k + 1) * kDt;
        out.finite = out.finite && all_finite(o);
        out.brake_max = std::max(out.brake_max, o.brake_applied);
        if (k + 1 == 500) out.vx05 = o.state.velocity.x();
        if (k + 1 == 1500) out.vx15 = o.state.velocity.x();
        if (t >= 0.5 - 1e-12 && t < 1.5 - 1e-12) {
            sum += o.ax;
            ++n;
        }
    }
    out.mean_ax = sum / n;
    return out;
}
}  // namespace

TEST(ReverseSlip, U3aReverseDecelerationAbsoluteInfo) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        const auto r = run_u3(p, +1.0);
        const bool within = std::abs(r.mean_ax - 1.0) <= 0.1;
        std::printf("U3a (information, PROP-2783) %s mean_ax=%+.6f vx(0.5)=%+.6f vx(1.5)=%+.6f "
                    "brake_max=%.6f absolute +1+-10%%=%s\n",
                    path_name(p), r.mean_ax, r.vx05, r.vx15, r.brake_max,
                    within ? "within" : "outside");
        EXPECT_TRUE(r.finite) << path_name(p);
        EXPECT_LT(std::abs(r.vx15), std::abs(r.vx05)) << path_name(p);
    }
}

TEST(ReverseSlip, U3aSymReverseDecelerationMirrorsForward) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        const auto rev = run_u3(p, +1.0, -3.0);
        const auto fwd = run_u3(p, -1.0, +3.0);
        const double mirror = -fwd.mean_ax;
        const double rel = (rev.mean_ax - mirror) / std::abs(fwd.mean_ax);
        std::printf("U3a-sym %s rev mean_ax=%+.6f fwd mean_ax=%+.6f rel=%+.4f %% "
                    "rev brake_max=%.6f rev vx(0.5)=%+.6f vx(1.5)=%+.6f fwd brake_max=%.6f\n",
                    path_name(p), rev.mean_ax, fwd.mean_ax, 100.0 * rel, rev.brake_max,
                    rev.vx05, rev.vx15, fwd.brake_max);
        EXPECT_TRUE(rev.finite && fwd.finite) << path_name(p);
        EXPECT_LE(std::abs(rev.mean_ax - mirror), 0.02 * std::abs(fwd.mean_ax)) << path_name(p);
        EXPECT_GT(rev.brake_max, 0.0) << path_name(p);
        EXPECT_LT(std::abs(rev.vx15), std::abs(rev.vx05)) << path_name(p);
    }
}

TEST(ReverseSlip, U3bReverseAccelerationCommandNoBrake) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        const auto r = run_u3(p, -0.5);
        std::printf("U3b %s mean_ax=%+.6f vx(0.5)=%+.6f vx(1.5)=%+.6f brake_max=%.6f\n",
                    path_name(p), r.mean_ax, r.vx05, r.vx15, r.brake_max);
        EXPECT_TRUE(r.finite) << path_name(p);
        EXPECT_EQ(r.brake_max, 0.0) << path_name(p);
        EXPECT_LT(r.mean_ax, 0.0) << path_name(p);
    }
}

// ---- U3d ----------------------------------------------------------------------------

namespace {
// The driver model reports a gear once (d) is in; before that the plant keeps +1.
template <class O, class = void> struct HasGear : std::false_type {};
template <class O>
struct HasGear<O, std::void_t<decltype(std::declval<const O&>().gear)>> : std::true_type {};
template <class O> int gear_of(const O& o) {
    if constexpr (HasGear<O>::value) return o.gear;
    else return 1;
}

DriverModel quiet_driver() {
    DriverModel d;
    DriverModel::Gains g;
    g.steer_noise_rms = 0.0;
    g.thr_noise_rms = 0.0;
    d.initialize(g);
    return d;
}

struct U3dRun {
    double mean_ax {0.0}, vx05 {0.0}, vx15 {0.0}, brake_max {0.0};
    int gear_min {1}, gear_max {1};
    bool finite {true};
};

U3dRun run_u3d(Path p, double vx0, double v_target) {
    const VehicleParams vp = sedan();
    auto s = session(p, vp, tire(true));
    s->reset_settled(start(vp, vx0, 0.0, 0.0));
    DriverModel d = quiet_driver();
    std::vector<double> px, py;
    for (int i = -200; i <= 200; ++i) { px.push_back(i); py.push_back(0.0); }
    U3dRun out;
    out.gear_min = 99;
    out.gear_max = -99;
    double sum = 0.0;
    int n = 0;
    for (int k = 0; k < 2000; ++k) {
        const auto o0 = s->output();
        const auto& st = o0.state;
        const double psi = std::atan2(2.0 * (st.orientation.w() * st.orientation.z() +
                                             st.orientation.x() * st.orientation.y()),
                                      1.0 - 2.0 * (st.orientation.y() * st.orientation.y() +
                                                   st.orientation.z() * st.orientation.z()));
        const auto dm = d.update(st.position.x(), st.position.y(), psi, st.velocity.x(),
                                 v_target, px.data(), py.data(), (int)px.size(), kDt);
        CmdL4 u;
        u.throttle = dm.throttle;
        u.brake = dm.brake;
        u.steer_angle_wheel = 0.0;
        u.gear = gear_of(dm);
        out.gear_min = std::min(out.gear_min, u.gear);
        out.gear_max = std::max(out.gear_max, u.gear);
        s->set_input(u);
        s->tick(kDt);
        const auto o = s->output();
        const double t = (k + 1) * kDt;
        out.finite = out.finite && all_finite(o);
        out.brake_max = std::max(out.brake_max, o.brake_applied);
        if (k + 1 == 500) out.vx05 = o.state.velocity.x();
        if (k + 1 == 1500) out.vx15 = o.state.velocity.x();
        if (t >= 0.5 - 1e-12 && t < 1.5 - 1e-12) {
            sum += o.ax;
            ++n;
        }
    }
    out.mean_ax = sum / n;
    return out;
}
}  // namespace

TEST(ReverseSlip, U3dDriverModelReverseAccelerationNoBrake) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        const auto r = run_u3d(p, -3.0, -5.0);
        std::printf("U3d(a) %s v_target=-5 mean_ax=%+.6f vx(0.5)=%+.6f vx(1.5)=%+.6f "
                    "brake_max=%.6f gear=[%d %d]\n",
                    path_name(p), r.mean_ax, r.vx05, r.vx15, r.brake_max, r.gear_min, r.gear_max);
        EXPECT_TRUE(r.finite) << path_name(p);
        EXPECT_EQ(r.brake_max, 0.0) << path_name(p);
        EXPECT_LT(r.mean_ax, 0.0) << path_name(p);
    }
}

TEST(ReverseSlip, U3dDriverModelReverseDecelerationSlowsDown) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        const auto r = run_u3d(p, -3.0, -1.0);
        std::printf("U3d(b) %s v_target=-1 mean_ax=%+.6f vx(0.5)=%+.6f vx(1.5)=%+.6f "
                    "brake_max=%.6f gear=[%d %d]\n",
                    path_name(p), r.mean_ax, r.vx05, r.vx15, r.brake_max, r.gear_min, r.gear_max);
        EXPECT_TRUE(r.finite) << path_name(p);
        EXPECT_LT(std::abs(r.vx15), std::abs(r.vx05)) << path_name(p);
    }
}

TEST(ReverseSlip, U3dDriverModelForwardMatchesPre57Chain) {
    // Pre-57 DriverModel longitudinal chain: vx PID -> LongAxController::update(ax, 0, dt).
    DriverModel::Gains g;
    g.steer_noise_rms = 0.0;
    g.thr_noise_rms = 0.0;
    std::vector<double> px, py;
    for (int i = -200; i <= 200; ++i) { px.push_back(i); py.push_back(0.0); }
    for (double v_target : {5.0, 1.0}) {
        DriverModel d;
        d.initialize(g);
        LongVxController vxc;
        LongVxController::Gains vg;
        vg.kp = g.vx_kp;
        vg.ki = g.vx_ki;
        vxc.initialize(vg);
        LongAxController axc;
        axc.initialize({});
        int mismatch = 0, gear_bad = 0;
        double vx = 3.0;
        for (int k = 0; k < 2000; ++k) {
            const auto dm = d.update(0.0, 0.0, 0.0, vx, v_target, px.data(), py.data(),
                                     (int)px.size(), kDt);
            const double ax_tgt = vxc.update(v_target, vx, kDt);
            const auto [thr, brk] = axc.update(ax_tgt, 0.0, kDt);
            const double want_thr = brk > 0.0 ? 0.0 : std::clamp(thr, 0.0, 1.0);
            const double want_brk = brk > 0.0 ? std::clamp(brk, 0.0, 1.0) : 0.0;
            if (std::memcmp(&dm.throttle, &want_thr, sizeof(double)) != 0 ||
                std::memcmp(&dm.brake, &want_brk, sizeof(double)) != 0)
                ++mismatch;
            if (gear_of(dm) != 1) ++gear_bad;
            vx += 0.5 * (dm.throttle - dm.brake) * kDt;  // any forward speed path
        }
        std::printf("U3d forward v_target=%+.1f ticks=2000 pedal_mismatch=%d gear_not_1=%d\n",
                    v_target, mismatch, gear_bad);
        EXPECT_EQ(mismatch, 0) << v_target;
        EXPECT_EQ(gear_bad, 0) << v_target;
    }
}

// ---- U4 -----------------------------------------------------------------------------

TEST(ReverseSlip, U4SideSlideAndReverseSpinFiniteAndFrictionBounded) {
    struct Case { const char* name; double vx, vy, r; };
    // 11-13 probe states A-H (VDSim-PO probe_rev.py) + I (probe_rev2 reverse straight).
    const std::vector<Case> cases = {
        {"A fwd slide",        3.6,  2.0,  0.0},
        {"B rev slide",       -3.6,  2.0,  0.0},
        {"C rev slide",       -3.6, -2.0,  0.0},
        {"D rev straight+",   -3.0,  0.01, 0.0},
        {"E rev straight-",   -3.0, -0.01, 0.0},
        {"F side vx0",         0.0,  3.0,  0.0},
        {"G side vx0.1",       0.1,  3.0,  0.0},
        {"H rev spin",        -3.6,  3.0,  1.5},
        {"I rev straight0",   -3.0,  0.0,  0.0},
    };
    const VehicleParams vp = sedan();
    const TireParams tp = tire(true);
    const double mu = tp.mu_nominal * 1.0;
    for (Path p : {Path::L2, Path::L2Plant, Path::L5Penalty}) {
        for (const auto& c : cases) {
            auto s = session(p, vp, tp);
            s->reset_settled(start(vp, c.vx, c.vy, c.r));
            bool finite = true;
            double worst = -1e300;  // max over ticks/wheels of |Fy| - mu Fz (allowance)
            int first_bad_tick = -1;
            for (int k = 0; k < 2000; ++k) {
                CmdL5 u;
                u.ax_target = 0.0;
                s->set_input(u);
                s->tick(kDt);
                const auto o = s->output();
                if (!all_finite(o)) {
                    finite = false;
                    if (first_bad_tick < 0) first_bad_tick = k + 1;
                    break;
                }
                for (int i = 0; i < NUM_WHEELS; ++i) {
                    const double lim = mu * o.Fz[i];
                    double fy = std::abs(o.tire_forces_wheel[i].y());
                    if (p == Path::L5Penalty) {
                        // L5 reports this field in the body frame (normal force included);
                        // judge the tyre's contact-plane force instead (>= its lateral part).
                        const Vec3 nb = o.state.orientation.toRotationMatrix().transpose() *
                                        Vec3(0.0, 0.0, 1.0);
                        const Vec3& F = o.tire_forces_wheel[i];
                        fy = (F - F.dot(nb) * nb).norm();
                    }
                    const double ex = fy - lim - (1e-9 * std::abs(lim) + 1e-6);
                    worst = std::max(worst, ex);
                }
            }
            const auto o = s->output();
            std::printf("U4 %s %-16s finite=%d first_nonfinite_tick=%d worst(|Fy|-muFz-slack)=%+.6e N "
                        "end vx=%+.4f vy=%+.4f r=%+.4f\n",
                        path_name(p), c.name, (int)finite, first_bad_tick, worst,
                        o.state.velocity.x(), o.state.velocity.y(), o.state.angular_velocity.z());
            EXPECT_TRUE(finite) << path_name(p) << " " << c.name;
            EXPECT_LE(worst, 0.0) << path_name(p) << " " << c.name;
        }
    }
}

// ---- U5 -----------------------------------------------------------------------------

namespace {
struct U5Run {
    bool finite {true}, stopped {false};
    double t_stop {-1.0}, hold_max_vx {0.0};
    std::array<int, NUM_WHEELS> sign_changes {{0, 0, 0, 0}};
    std::vector<double> bits;  // full state trace for off == unset
};

U5Run run_u5(Path p, const std::string& key) {
    const VehicleParams vp = sedan(key);
    auto s = session(p, vp, tire(true));
    s->reset_settled(start(vp, 5.0, 0.0, 0.0));
    U5Run out;
    std::array<int, NUM_WHEELS> last_sign {{0, 0, 0, 0}};
    int k_stop = -1;
    for (int k = 0; k < 9000; ++k) {
        CmdL4 u;
        u.throttle = 0.0;
        u.brake = 0.3;
        s->set_input(u);
        s->tick(kDt);
        const auto o = s->output();
        out.finite = out.finite && all_finite(o);
        const auto& st = o.state;
        for (double v : {st.position.x(), st.position.y(), st.position.z(), st.velocity.x(),
                         st.velocity.y(), st.velocity.z(), st.angular_velocity.x(),
                         st.angular_velocity.y(), st.angular_velocity.z()})
            out.bits.push_back(v);
        for (int i = 0; i < NUM_WHEELS; ++i) {
            out.bits.push_back(st.wheel_spin[i]);
            out.bits.push_back(o.tire_forces_wheel[i].x());
        }
        const double vx = st.velocity.x();
        if (k_stop < 0) {
            if (std::abs(vx) < 0.01) {
                k_stop = k;
                out.t_stop = (k + 1) * kDt;
                out.stopped = true;
            } else if (k + 1 >= 6000) {
                break;  // no stop within 6 s
            }
            continue;
        }
        const int since = k - k_stop;  // 1..3000 after the stop tick
        out.hold_max_vx = std::max(out.hold_max_vx, std::abs(vx));
        if (since > 2000) {
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const double fx = o.tire_forces_wheel[i].x();
                const int sg = fx > 0.0 ? 1 : (fx < 0.0 ? -1 : 0);
                if (sg != 0) {
                    if (last_sign[i] != 0 && sg != last_sign[i]) ++out.sign_changes[i];
                    last_sign[i] = sg;
                }
            }
        }
        if (since >= 3000) break;
    }
    return out;
}
}  // namespace

TEST(ReverseSlip, U5RelaxedHoldsStandstillWithoutChatter) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        const auto r = run_u5(p, "relaxed");
        std::printf("U5 %s relaxed stopped=%d t_stop=%.3f hold_max|vx|=%.3e sign_changes=[%d %d %d %d] finite=%d\n",
                    path_name(p), (int)r.stopped, r.t_stop, r.hold_max_vx, r.sign_changes[0],
                    r.sign_changes[1], r.sign_changes[2], r.sign_changes[3], (int)r.finite);
        EXPECT_TRUE(r.finite) << path_name(p);
        ASSERT_TRUE(r.stopped) << path_name(p) << " no stop within 6 s";
        EXPECT_LE(r.hold_max_vx, 0.05) << path_name(p);
        for (int i = 0; i < NUM_WHEELS; ++i)
            EXPECT_LE(r.sign_changes[i], 2) << path_name(p) << " wheel " << i;
    }
}

TEST(ReverseSlip, U5OffEqualsUnsetBitForBit) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        const auto a = run_u5(p, "");
        const auto b = run_u5(p, "off");
        ASSERT_EQ(a.bits.size(), b.bits.size()) << path_name(p);
        EXPECT_EQ(std::memcmp(a.bits.data(), b.bits.data(), a.bits.size() * sizeof(double)), 0)
            << path_name(p);
        std::printf("U5 %s off==unset values=%zu t_stop=%.3f sign_changes(unset)=[%d %d %d %d] (info)\n",
                    path_name(p), a.bits.size(), a.t_stop, a.sign_changes[0], a.sign_changes[1],
                    a.sign_changes[2], a.sign_changes[3]);
    }
}

// ---- K1 -----------------------------------------------------------------------------

static_assert(kRelaxedSigmaLat == 0.6, "relaxed sigma_y fixed by the 57 gate commit");
static_assert(kRelaxedSigmaLong == 0.15, "relaxed sigma_x fixed by the 57 gate commit");
static_assert(kRelaxedVLow == 0.15, "relaxed v_low fixed by the 57 gate commit");

namespace {
struct K1Run {
    double tau {0.0}, t_state {-1.0}, t_force {-1.0};
};

/// First time a monotone-ish record reaches `level` (linear interpolation), -1 if never.
double first_crossing(const std::vector<double>& t, const std::vector<double>& v, double level) {
    for (std::size_t k = 1; k < v.size(); ++k) {
        const bool up = level >= v[0];
        const bool hit = up ? (v[k] >= level) : (v[k] <= level);
        if (hit) {
            const double a = v[k - 1], b = v[k];
            const double f = (b == a) ? 1.0 : (level - a) / (b - a);
            return t[k - 1] + f * (t[k] - t[k - 1]);
        }
    }
    return -1.0;
}

/// lateral = alpha 1 deg step, else kappa 0.02 step. vx signed.
K1Run run_k1(bool lugre, bool lateral, double vx) {
    TireParams tp = tire(lugre);
    tp.belt.enabled = true;
    tp.belt.sigma_lat = kRelaxedSigmaLat;
    tp.belt.sigma_long = kRelaxedSigmaLong;
    const VehicleParams vp = sedan();
    auto tm = make_tire_model(tp);
    tm->initialize(tp);
    ITireModel::ContactInput ci;
    ci.Fz = vp.mass * 9.81 / 4.0;
    ci.Vx = vx;
    ci.Vy = 0.0;
    ci.R0 = vp.wheel_radius_nominal;
    ci.omega = vx / ci.R0;
    ITireModel::Transient tr;
    const double Re = tm->evaluate(ci, tr).Re;
    ci.omega = vx / Re;
    const double sigma = lateral ? kRelaxedSigmaLat : kRelaxedSigmaLong;
    K1Run out;
    out.tau = sigma / std::abs(vx);
    // step
    if (lateral) ci.Vy = std::abs(vx) * std::tan(kPi / 180.0);
    else ci.omega = (vx + 0.02 * std::abs(vx)) / Re;
    const double dt = out.tau / 200.0;
    std::vector<double> ts {0.0}, st {0.0}, fo;
    fo.push_back(lateral ? tm->evaluate(ci, tr).Fy : tm->evaluate(ci, tr).Fx);
    for (int k = 1; k <= 3000; ++k) {  // 15 tau
        tr = tm->advance_relaxation(ci, tr, dt);
        if (lugre)
            for (int j = 0; j < 20; ++j) tr = tm->advance_bristle(ci, tr, dt / 20.0);
        ts.push_back(k * dt);
        st.push_back(lugre ? (lateral ? tr.belt_vlat : tr.belt_vlong)
                           : (lateral ? tr.belt_alpha : tr.belt_kappa));
        const auto w = tm->evaluate(ci, tr);
        fo.push_back(lateral ? w.Fy : w.Fx);
    }
    const auto ck = tire_contact_kinematics(ci.Vx, ci.Vy, ci.omega, ci.Fz, 0.0, tp, ci.R0);
    const double target = lugre ? (lateral ? ck.vsy : ck.vsx) : (lateral ? ck.alpha : ck.kappa);
    const double frac = 1.0 - std::exp(-1.0);
    out.t_state = first_crossing(ts, st, frac * target);
    out.t_force = first_crossing(ts, fo, fo[0] + frac * (fo.back() - fo[0]));
    return out;
}
}  // namespace

TEST(ReverseSlipLowSpeed, K1RelaxationTimeConstantTyreAlone) {
    for (bool lugre : {false, true}) {
        for (bool lateral : {true, false}) {
            for (double vx : {5.0, 10.0, -5.0, -10.0}) {
                const auto r = run_k1(lugre, lateral, vx);
                const double rel = r.t_state / r.tau - 1.0;
                std::printf("K1 %s %s vx=%+5.1f tau=%.6f t632_state=%.6f rel=%+.4f %% "
                            "t632_force=%.6f force_rel=%+.4f %% (info)\n",
                            lugre ? "LuGre" : "MF   ", lateral ? "alpha" : "kappa", vx, r.tau,
                            r.t_state, 100.0 * rel, r.t_force, 100.0 * (r.t_force / r.tau - 1.0));
                EXPECT_GT(r.t_state, 0.0) << lugre << lateral << vx;
                EXPECT_LE(std::abs(rel), 0.05) << lugre << lateral << vx;
            }
        }
    }
}

// ---- K2 -----------------------------------------------------------------------------

namespace {
struct K2Run {
    bool finite {true};
    double vx_end {0.0};
    std::array<int, NUM_WHEELS> fx_changes {{0, 0, 0, 0}}, fy_changes {{0, 0, 0, 0}};
};

struct SignCount {
    std::array<int, NUM_WHEELS> last {{0, 0, 0, 0}};
    void add(int i, double v, std::array<int, NUM_WHEELS>& changes) {
        const int sg = v > 0.0 ? 1 : (v < 0.0 ? -1 : 0);
        if (sg == 0) return;
        if (last[i] != 0 && sg != last[i]) ++changes[i];
        last[i] = sg;
    }
};

/// cond 1 = C1 (decelerate, then a_cmd 0 for 3 s), 3 = C3 (state F, 3 s).
K2Run run_k2(Path p, const std::string& key, int cond) {
    const VehicleParams vp = sedan(key);
    auto s = session(p, vp, tire(true));
    s->reset_settled(cond == 1 ? start(vp, 5.0, 0.0, 0.0) : start(vp, 0.0, 3.0, 0.0));
    K2Run out;
    SignCount cx, cy;
    int k_sw = (cond == 3) ? -2 : -1;  // C3 has no switch; its last 1 s is (2, 3] s
    for (int k = 0; k < 20000; ++k) {
        CmdL5 u;
        u.ax_target = (cond == 1 && k_sw < 0) ? -1.0 : 0.0;
        s->set_input(u);
        s->tick(kDt);
        const auto o = s->output();
        out.finite = out.finite && all_finite(o);
        out.vx_end = o.state.velocity.x();
        int since;
        if (cond == 1) {
            if (k_sw < 0) {
                if (std::abs(out.vx_end) < 0.1) k_sw = k;
                continue;
            }
            since = k - k_sw;
        } else {
            since = k + 1;
        }
        if (since > 2000)
            for (int i = 0; i < NUM_WHEELS; ++i) {
                cx.add(i, o.tire_forces_wheel[i].x(), out.fx_changes);
                cy.add(i, o.tire_forces_wheel[i].y(), out.fy_changes);
            }
        if (since >= 3000) break;
    }
    return out;
}
}  // namespace

TEST(ReverseSlipLowSpeed, K2RelaxedNoMoreChatterThanOffAndHolds) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        for (int cond : {1, 3}) {
            const auto off = run_k2(p, "", cond);
            const auto rel = run_k2(p, "relaxed", cond);
            std::printf("K2 %s C%d off Fx_changes=[%d %d %d %d] relaxed Fx_changes=[%d %d %d %d] "
                        "relaxed vx_end=%+.3e off vx_end=%+.3e finite=%d "
                        "Fy_changes off=[%d %d %d %d] relaxed=[%d %d %d %d] (Fy info)\n",
                        path_name(p), cond, off.fx_changes[0], off.fx_changes[1],
                        off.fx_changes[2], off.fx_changes[3], rel.fx_changes[0],
                        rel.fx_changes[1], rel.fx_changes[2], rel.fx_changes[3], rel.vx_end,
                        off.vx_end, (int)rel.finite, off.fy_changes[0], off.fy_changes[1],
                        off.fy_changes[2], off.fy_changes[3], rel.fy_changes[0],
                        rel.fy_changes[1], rel.fy_changes[2], rel.fy_changes[3]);
            EXPECT_TRUE(rel.finite) << path_name(p) << " C" << cond;
            EXPECT_LT(std::abs(rel.vx_end), 0.01) << path_name(p) << " C" << cond;
            for (int i = 0; i < NUM_WHEELS; ++i)
                EXPECT_LE(rel.fx_changes[i], off.fx_changes[i])
                    << path_name(p) << " C" << cond << " wheel " << i;
        }
    }
}

// ---- K4b ----------------------------------------------------------------------------

namespace {
double k4b_mean_ay(Path p, const std::string& key, bool& finite) {
    const VehicleParams vp = sedan(key);
    auto s = session(p, vp, tire(true));
    s->reset_settled(start(vp, 10.0, 0.0, 0.0));
    double sum = 0.0;
    int n = 0;
    finite = true;
    for (int k = 0; k < 150; ++k) {
        CmdL4 u;
        u.steer_angle_wheel = (k + 1) * kDt > 0.1 + 1e-12 ? 0.05 : 0.0;
        s->set_input(u);
        s->tick(kDt);
        const auto o = s->output();
        finite = finite && all_finite(o);
        const double t = (k + 1) * kDt;
        if (t > 0.1 + 1e-12 && t <= 0.15 + 1e-12) {
            sum += std::abs(o.ay);
            ++n;
        }
    }
    return sum / n;
}
}  // namespace

TEST(ReverseSlipLowSpeed, K4bRelaxedKeyDelaysLateralForceOnSession) {
    for (Path p : {Path::L2, Path::L5Penalty}) {
        bool fin_off = true, fin_rel = true;
        const double off = k4b_mean_ay(p, "", fin_off);
        const double rel = k4b_mean_ay(p, "relaxed", fin_rel);
        std::printf("K4b %s mean|ay|(0.10,0.15] off=%.6f relaxed=%.6f ratio=%.4f\n", path_name(p),
                    off, rel, rel / off);
        EXPECT_TRUE(fin_off && fin_rel) << path_name(p);
        EXPECT_LT(rel, off) << path_name(p);
    }
}

// ---- key contract -------------------------------------------------------------------

TEST(ReverseSlipKeys, TireLowSpeedYamlRoundTripAndUnsetWritesNoKey) {
    for (const char* v : {"off", "relaxed"}) {
        const auto vp = sedan(v);
        const auto out = ::testing::TempDir() + std::string("u57_out_") + v + ".yaml";
        vp.to_yaml(out);
        const auto text = read_all(out);
        // The key line may carry a quoted scalar ("off" is YAML 1.1 bool-like).
        const auto at = text.find("\ntire_low_speed:");
        ASSERT_NE(at, std::string::npos) << v;
        const auto line = text.substr(at + 1, text.find('\n', at + 1) - at - 1);
        EXPECT_NE(line.find(v), std::string::npos) << v << " line: " << line;
        const auto again = ::testing::TempDir() + std::string("u57_again_") + v + ".yaml";
        VehicleParams::from_yaml(out).to_yaml(again);
        EXPECT_EQ(read_all(again), text) << v;
    }
    const auto plain = ::testing::TempDir() + "u57_plain.yaml";
    sedan().to_yaml(plain);
    EXPECT_EQ(read_all(plain).find("tire_low_speed"), std::string::npos);
}

TEST(ReverseSlipKeys, TireLowSpeedUnknownValueThrows) {
    for (const char* v : {"Relaxed", "on", "1"}) {
        EXPECT_THROW(sedan(v), std::invalid_argument) << v;
    }
}
