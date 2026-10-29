// L5 drive / brake torque reactions (work order 43, 0.8.1): R1, R2, R3, R6.
//
// Gates fixed before measurement (PO 1st verdict, Q1 / Q2 / J1 / J8):
//   axle load transfer  dFz_front = -(m * ax_meas * h_x + sum_i I_eff,i * alpha_i) / L,
//                       dFz_rear  = -dFz_front, +/-2 % per axle;
//   h_x = cg_height on the penalty path, h_sys = ((m - sum m_u) h + sum m_u R) / m on the
//   strut path (the body there is the sprung mass at cg_height, the four wheel
//   particles ride at R); I_eff = I_wheel + reflected engine share; alpha = window
//   mean wheel-spin acceleration.
// Manoeuvre: flat ground mu 1.0, no aero, generic_pacejka (rolling resistance 0),
// 1 ms ticks, CmdL4 step at t = 2.0 s (throttle 0.4 from 10 m/s, or brake 0.4 from
// 25 m/s; 20 m/s for the FS car). One window [4.0, 5.0) s, no search. The window is
// valid only if the 50 ms moving average of ax has a regression slope
// |dax/dt| <= 0.10 m/s^3 and the mean |pitch rate| <= 2e-3 rad/s; otherwise the test
// fails (stop and report). Static reference: the same car at rest, mean over [1, 2) s.
//
// Acceleration in every expectation (PO 3rd verdict G1-G3, changed after measurement):
// a_g = least-squares slope over the window of the CG velocity in world coordinates,
// horizontal component along the heading. The body output ax is the body-frame specific
// force and carries g * sin(theta) at window pitch theta, so it is not the horizontal
// inertial acceleration of m * a * h / L. ax stays as an info column (original formula),
// and still drives the window steadiness check above (a validity condition, not an
// expectation). a_pos = 0.5 s second difference of the heading-projected CG position
// (the D4 probe) is info only.
#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/drivetrain_inertia.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
constexpr double kDt = 1e-3;
constexpr int kStepTick = 2000;                 // command step at 2.0 s
constexpr int kWin0 = 4000, kWin1 = 5000;       // window [4.0, 5.0) s
constexpr int kMaWin = 50;                      // 50 ms moving average of ax
constexpr double kMaxAxSlope = 0.10;            // [m/s^3]
constexpr double kMaxPitchRate = 2e-3;          // [rad/s]

enum class Car { Sedan, FS };
enum class Path { Penalty, Strut, StrutDae };
enum class Move { Accel, Brake };

VehicleParams car(Car c) {
    VehicleParams vp = VehicleParams::from_yaml(
        kSrc + (c == Car::Sedan ? "/configs/vehicles/generic_sedan.yaml"
                                : "/configs/vehicles/fs_aero_example.yaml"));
    vp.aero_elements.clear();      // no aero at all
    vp.aero_drag_coeff = 0.0;
    vp.aero_lift_front = vp.aero_lift_rear = 0.0;
    return vp;
}

TireParams tire() {
    return TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
}

/// Test-only DW sample: dw_front_sports with the UCA chassis pivots at z 0.35 / 0.29.
std::string dw_sample() { return kSrc + "/tests/fixtures/dw_front_antidive_sample.yaml"; }
std::string ta_rear() { return kSrc + "/configs/parts/susp_kinematics/kin/ta_rear_sedan.yaml"; }

double m_u(const VehicleParams& vp, int i) { return std::max(1.0, vp.unsprung_mass[i]); }

double sum_m_u(const VehicleParams& vp) {
    double s = 0.0;
    for (int i = 0; i < NUM_WHEELS; ++i) s += m_u(vp, i);
    return s;
}

/// Q2: pitch arm of the whole vehicle on each path.
double h_x(const VehicleParams& vp, Path p) {
    if (p == Path::Penalty) return vp.cg_height;
    const double mu = sum_m_u(vp);
    return ((vp.mass - mu) * vp.cg_height + mu * vp.wheel_radius_nominal) / vp.mass;
}

/// Wheel spin inertia as the L5 core derives it (wheel_inertia 0 -> solid disc).
double I_wheel(const VehicleParams& vp, int i) {
    if (vp.wheel_inertia[i] > 0.0) return vp.wheel_inertia[i];
    const double mw = vp.unsprung_mass[i] > 0.0 ? vp.unsprung_mass[i] : 25.0;
    const double R = vp.wheel_radius_nominal;
    return std::max(0.01, 0.5 * mw * R * R);
}

/// Reflected engine share (BasicDrivetrain; the cars here have no powertrain block).
double I_eng(const VehicleParams& vp, int i) { return wheel_engine_inertia_share(vp, i); }

SolverParams solver(Path p) {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    sp.l5_spatial_suspension = (p != Path::Penalty);
    return sp;
}

struct Window {
    double front {0.0}, rear {0.0};               // axle Fz [N]
    double ax {0.0};                               // body specific force x [m/s^2] (info)
    double a_g {0.0};                              // G1 ground horizontal CG accel [m/s^2]
    double a_pos {0.0};                            // info: 0.5 s position 2nd diff [m/s^2]
    std::array<double, NUM_WHEELS> alpha {};       // wheel-spin accel [rad/s^2]
    std::array<double, NUM_WHEELS> conn_x {};      // tire_forces_body x [N]
    std::array<double, NUM_WHEELS> spring {};      // strut spring + stop [N]
    std::array<double, NUM_WHEELS> travel {};      // susp_compression [m]
    double ax_slope {0.0};                         // [m/s^3]
    double pitch_rate {0.0};                       // mean |omega_y| [rad/s]
    bool finite {true};
};

/// Least-squares slope dy/dt.
double slope(const std::vector<double>& t, const std::vector<double>& y) {
    double mt = 0.0, my = 0.0;
    for (std::size_t j = 0; j < t.size(); ++j) { mt += t[j]; my += y[j]; }
    mt /= t.size(); my /= y.size();
    double sxy = 0.0, sxx = 0.0;
    for (std::size_t j = 0; j < t.size(); ++j) {
        sxy += (t[j] - mt) * (y[j] - my);
        sxx += (t[j] - mt) * (t[j] - mt);
    }
    return sxy / sxx;
}

/// One run. move == nullptr -> the static reference (at rest, mean over [1, 2) s).
Window run(const VehicleParams& vp, Path p, const Move* move, double v0) {
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    SimSession s(create_stunt_dof(), create_flat_ground(0.0, 1.0), vp, TireSetup(tire()),
                 solver(p), cfg);
    if (p == Path::StrutDae) {
        free_3d_attach_multibody(s.dynamics(), true,
                                 mb::SuspensionTopology::from_yaml(dw_sample()), true);
        free_3d_attach_multibody(s.dynamics(), false,
                                 mb::SuspensionTopology::from_yaml(ta_rear()), true);
    }
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = v0;
    const double w = v0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    s.reset_settled(s0);

    const int w0 = move ? kWin0 : 1000, w1 = move ? kWin1 : 2000;
    Window r;
    std::vector<double> ax_hist, ma_t, ma_v;
    std::array<double, NUM_WHEELS> spin0 {};
    std::vector<double> vg_t, vg_v;                 // heading-projected world CG velocity
    Vec3 head {1.0, 0.0, 0.0};
    double x0 = 0.0, xm = 0.0;
    for (int k = 0; k < w1; ++k) {
        CmdL4 u;
        if (move && k >= kStepTick) {
            if (*move == Move::Accel) u.throttle = 0.4;
            else u.brake = 0.4;
        }
        s.set_input(u);
        s.tick(kDt);
        const auto o = s.output();
        ax_hist.push_back(o.ax);
        if (k == w0 - 1) {
            spin0 = o.state.wheel_spin;
            const double psi = o.state.yaw();
            head = Vec3(std::cos(psi), std::sin(psi), 0.0);
            x0 = o.state.position.dot(head);
        }
        if (k == (w0 + w1) / 2 - 1) xm = o.state.position.dot(head);
        if (k < w0) continue;
        const Vec3 v_world = o.state.orientation.toRotationMatrix() * o.state.velocity;
        vg_t.push_back((k + 1) * kDt);
        vg_v.push_back(v_world.dot(head));
        r.front += o.Fz[WHEEL_FL] + o.Fz[WHEEL_FR];
        r.rear  += o.Fz[WHEEL_RL] + o.Fz[WHEEL_RR];
        r.ax += o.ax;
        r.pitch_rate += std::abs(o.state.angular_velocity.y());
        const auto F = s.dynamics().tire_forces_body();
        const auto sf = free_3d_strut_force(s.dynamics());
        for (int i = 0; i < NUM_WHEELS; ++i) {
            r.conn_x[i] += F[i].x();
            r.spring[i] += sf[i];
            r.travel[i] += o.state.susp_compression[i];
        }
        double ma = 0.0;
        for (int j = 0; j < kMaWin; ++j) ma += ax_hist[ax_hist.size() - 1 - j];
        ma_t.push_back((k + 1) * kDt);
        ma_v.push_back(ma / kMaWin);
        r.finite = r.finite && std::isfinite(o.Fz[0] + o.Fz[1] + o.Fz[2] + o.Fz[3] + o.ax);
        if (k == w1 - 1) {
            const double T = (w1 - w0) * kDt;
            for (int i = 0; i < NUM_WHEELS; ++i)
                r.alpha[i] = (o.state.wheel_spin[i] - spin0[i]) / T;
            const double half = 0.5 * T;
            r.a_pos = (o.state.position.dot(head) - 2.0 * xm + x0) / (half * half);
        }
    }
    const double n = static_cast<double>(w1 - w0);
    r.front /= n; r.rear /= n; r.ax /= n; r.pitch_rate /= n;
    for (int i = 0; i < NUM_WHEELS; ++i) {
        r.conn_x[i] /= n; r.spring[i] /= n; r.travel[i] /= n;
    }
    r.a_g = slope(vg_t, vg_v);
    // Least-squares slope of the moving average over the window.
    double mt = 0.0, mv = 0.0;
    for (std::size_t j = 0; j < ma_t.size(); ++j) { mt += ma_t[j]; mv += ma_v[j]; }
    mt /= ma_t.size(); mv /= ma_v.size();
    double sxy = 0.0, sxx = 0.0;
    for (std::size_t j = 0; j < ma_t.size(); ++j) {
        sxy += (ma_t[j] - mt) * (ma_v[j] - mv);
        sxx += (ma_t[j] - mt) * (ma_t[j] - mt);
    }
    r.ax_slope = sxy / sxx;
    return r;
}

double v_start(Car c, Move m) {
    if (m == Move::Accel) return 10.0;
    return c == Car::Sedan ? 25.0 : 20.0;
}

struct Transfer {
    double d_front, d_rear;     // measured axle dFz [N]
    double want_front;          // Q1 expectation, front [N]; rear = -want_front
    double sum_I_alpha;         // [N m]
    double moment_lhs;          // R6: -(dF_front * a - dF_rear * b) [N m]
    double moment_rhs;          // R6: m * a_g * h_x + sum I_eff alpha [N m]
    Window win, stat;
};

/// Run one condition plus its static reference and print the J1 info columns.
Transfer transfer(const char* tag, const VehicleParams& vp, Path p, Move m, double v0) {
    Transfer t;
    t.stat = run(vp, p, nullptr, 0.0);
    t.win = run(vp, p, &m, v0);
    const auto& w = t.win;
    t.d_front = w.front - t.stat.front;
    t.d_rear  = w.rear - t.stat.rear;
    double sIa = 0.0, sIeng = 0.0, sIeff = 0.0;
    for (int i = 0; i < NUM_WHEELS; ++i) {
        const double Ie = I_wheel(vp, i) + I_eng(vp, i);
        sIa += Ie * w.alpha[i];
        sIeng += I_eng(vp, i);
        sIeff += Ie;
    }
    t.sum_I_alpha = sIa;
    const double h = h_x(vp, p);
    const double L = vp.wheelbase;
    t.want_front = -(vp.mass * w.a_g * h + sIa) / L;
    t.moment_lhs = -(t.d_front * vp.cg_to_front - t.d_rear * vp.cg_to_rear);
    t.moment_rhs = vp.mass * w.a_g * h + sIa;
    const double original = -vp.mass * w.ax * vp.cg_height / L;   // work-order formula
    const double want_ax = -(vp.mass * w.ax * h + sIa) / L;       // info: pre-G1 gate
    std::printf("[%s] a_g %.5f m/s^2 (ax %.5f, a_pos %.5f) slope %.4f m/s^3 |q| %.2e rad/s | "
                "dFz front %.2f (want %.2f) rear %.2f (want %.2f) N | h_x %.4f m, "
                "sum I*alpha %.3f N m | J1a front/original %.4f | J1b engine share %.4f | "
                "R6 %.3f vs %.3f N m\n",
                tag, w.a_g, w.ax, w.a_pos, w.ax_slope, w.pitch_rate, t.d_front,
                t.want_front, t.d_rear, -t.want_front, h, sIa, t.d_front / original,
                sIeng / sIeff, t.moment_lhs, t.moment_rhs);
    std::printf("[%s info] pre-G1 ax gate: want front %.2f N, resid %.2f %%\n", tag, want_ax,
                100.0 * (t.d_front - want_ax) / std::abs(want_ax));
    return t;
}

void expect_window_valid(const Transfer& t, const char* tag) {
    ASSERT_TRUE(t.win.finite && t.stat.finite) << tag;
    EXPECT_LE(std::abs(t.win.ax_slope), kMaxAxSlope) << tag << " window not steady";
    EXPECT_LE(t.win.pitch_rate, kMaxPitchRate) << tag << " window not steady";
}

void expect_transfer(const Transfer& t, const char* tag) {
    expect_window_valid(t, tag);
    EXPECT_NEAR(t.d_front, t.want_front, 0.02 * std::abs(t.want_front)) << tag;
    EXPECT_NEAR(t.d_rear, -t.want_front, 0.02 * std::abs(t.want_front)) << tag;
}

}  // namespace

// ---- R1: penalty path, generic_sedan and fs_aero_example (aero emptied) ----
TEST(L5Reaction, R1_PenaltySedanAccel) {
    expect_transfer(transfer("R1 sedan accel", car(Car::Sedan), Path::Penalty, Move::Accel,
                             v_start(Car::Sedan, Move::Accel)), "R1 sedan accel");
}
TEST(L5Reaction, R1_PenaltySedanBrake) {
    expect_transfer(transfer("R1 sedan brake", car(Car::Sedan), Path::Penalty, Move::Brake,
                             v_start(Car::Sedan, Move::Brake)), "R1 sedan brake");
}
TEST(L5Reaction, R1_PenaltyFSAccel) {
    expect_transfer(transfer("R1 FS accel", car(Car::FS), Path::Penalty, Move::Accel,
                             v_start(Car::FS, Move::Accel)), "R1 FS accel");
}
TEST(L5Reaction, R1_PenaltyFSBrake) {
    expect_transfer(transfer("R1 FS brake", car(Car::FS), Path::Penalty, Move::Brake,
                             v_start(Car::FS, Move::Brake)), "R1 FS brake");
}

// ---- R6: whole-vehicle pitch balance on the R1 runs, +/-2 % ----
TEST(L5Reaction, R6_PitchMomentBalance) {
    for (const Car c : {Car::Sedan, Car::FS}) {
        for (const Move m : {Move::Accel, Move::Brake}) {
            const auto t = transfer("R6", car(c), Path::Penalty, m, v_start(c, m));
            expect_window_valid(t, "R6");
            EXPECT_NEAR(t.moment_lhs, t.moment_rhs, 0.02 * std::abs(t.moment_rhs));
        }
    }
}

// ---- R2: strut path without a corner model, sprung vs unsprung ----
namespace {
void r2(Move m) {
    const char* tag[2] = {"R2 sprung", "R2 unsprung"};
    double d[2][2];
    for (int k = 0; k < 2; ++k) {
        auto vp = car(Car::Sedan);
        const auto t = k == 0 ? VehicleParams::TorqueReaction::Sprung
                              : VehicleParams::TorqueReaction::Unsprung;
        if (m == Move::Accel) vp.drive_reaction_rear = t;
        else vp.brake_reaction_front = vp.brake_reaction_rear = t;
        const auto tr = transfer(tag[k], vp, Path::Strut, m, v_start(Car::Sedan, m));
        expect_transfer(tr, tag[k]);
        d[k][0] = tr.d_front;
        d[k][1] = tr.d_rear;
    }
    EXPECT_LE(std::abs(d[1][0] - d[0][0]), 0.01 * std::abs(d[0][0]));
    EXPECT_LE(std::abs(d[1][1] - d[0][1]), 0.01 * std::abs(d[0][1]));
}
}  // namespace
TEST(L5Reaction, R2_StrutAccelSprungVsUnsprung) { r2(Move::Accel); }
TEST(L5Reaction, R2_StrutBrakeSprungVsUnsprung) { r2(Move::Brake); }

// ---- R3: strut + corner model, anti geometry (static instant-centre hand calc) ----
// Spring change of one axle (sum of its two struts):
//   dF_spring = dFz_axle + (sum Fx_axle - sum m_u * a_g) * dx/dz [+ sum M_i * theta'],
//   M_i = -(Fx_i * R + I_eff,i * alpha_i) on the knuckle (unsprung only).
// Front DW sample (PO 2nd-resume verdict, changed after measurement): dx/dz -0.039112 and
// theta' -0.353893 rad/m from the independent linkage solve
// tests/validation/analytical/l5_knuckle_reference.py (LK-UK-TK knuckle frame omega_y,
// +/-1 mm); the side-view hand calc (dx/dz -0.0896, theta' -0.4975) is a projection of the
// 3D linkage and is no longer the reference. Rear TA dx/dz +0.1891, theta' +1.5625 rad/m
// (hand calc, equal to the same script). Gate +/-5 %.
// Fx_i: on the strut path tire_forces_body() is the mount connection force, so the
// window-mean tyre force is that plus m_u * a_g (particle inertia); with an unsprung
// reaction the particle also carries F_eq = M * theta' * t / |t|^2, solved as
// Fx = (C + I_eff * alpha * k) / (1 - R k), C = conn_x + m_u * a_g, k = theta' t_x / |t|^2.
namespace {
struct Axle { int l, r; double dxdz, theta_p; };
constexpr Axle kFront {WHEEL_FL, WHEEL_FR, -0.039112, -0.353893};
constexpr Axle kRear  {WHEEL_RL, WHEEL_RR, +0.1891, +1.5625};

void r3(const char* tag, Move m, const Axle& ax_, bool unsprung) {
    auto vp = car(Car::Sedan);
    const auto t = unsprung ? VehicleParams::TorqueReaction::Unsprung
                            : VehicleParams::TorqueReaction::Sprung;
    if (m == Move::Brake) vp.brake_reaction_front = t;
    else vp.drive_reaction_rear = t;
    const auto tr = transfer(tag, vp, Path::StrutDae, m, v_start(Car::Sedan, m));
    expect_transfer(tr, tag);
    const auto& w = tr.win;
    const double R = vp.wheel_radius_nominal;
    const double tt = 1.0 + ax_.dxdz * ax_.dxdz;
    const double k = ax_.theta_p * ax_.dxdz / tt;
    double sFx = 0.0, sMu = 0.0, sM = 0.0, d_spring = 0.0, travel = 0.0;
    for (const int i : {ax_.l, ax_.r}) {
        const double Ie = I_wheel(vp, i) + I_eng(vp, i);
        const double C = w.conn_x[i] + m_u(vp, i) * w.a_g;
        const double Fx = unsprung ? (C + Ie * w.alpha[i] * k) / (1.0 - R * k) : C;
        sFx += Fx;
        sMu += m_u(vp, i);
        sM += -(Fx * R + Ie * w.alpha[i]);
        d_spring += w.spring[i] - tr.stat.spring[i];
        travel = std::max(travel, std::abs(w.travel[i] - tr.stat.travel[i]));
    }
    const double dFz = (ax_.l == WHEEL_FL) ? tr.d_front : tr.d_rear;
    const double want = dFz + (sFx - sMu * w.a_g) * ax_.dxdz
                      + (unsprung ? sM * ax_.theta_p : 0.0);
    const double anti = 1.0 - want / dFz;
    std::printf("[%s] dF_spring %.2f N (want %.2f) | anti %.4f | sum Fx %.2f N | "
                "max travel %.2f mm%s\n",
                tag, d_spring, want, anti, sFx, 1e3 * travel,
                travel > 5e-3 ? " (> 5 mm: static-IC gate, moved IC is info only)" : "");
    EXPECT_NEAR(d_spring, want, 0.05 * std::abs(want)) << tag;
}
}  // namespace
TEST(L5Reaction, R3_FrontBrakeSprung)   { r3("R3 front brake sprung",   Move::Brake, kFront, false); }
TEST(L5Reaction, R3_FrontBrakeUnsprung) { r3("R3 front brake unsprung", Move::Brake, kFront, true); }
TEST(L5Reaction, R3_RearAccelSprung)    { r3("R3 rear accel sprung",    Move::Accel, kRear,  false); }
TEST(L5Reaction, R3_RearAccelUnsprung)  { r3("R3 rear accel unsprung",  Move::Accel, kRear,  true); }
