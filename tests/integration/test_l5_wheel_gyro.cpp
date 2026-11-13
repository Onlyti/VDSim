// Work order 50 (wheel_gyro), integration gates. Fixed before the implementation commit.
//
// V2 (sim) : unset == legacy and full == legacy bit for bit where the camber is zero
//            (L5 penalty, L5 strut without a corner model). Wiring check on the corner-
//            model (DAE) path: full != legacy once the DAE camber is non-zero.
// V3       : steady skidpad, R 8.5 m, V 7 m/s (curvature and speed PI hold), generic
//            sedan, aero off, last 2 s mean. Measured off->full roll moment difference
//            about the whole-vehicle CG, a_y-normalised, against the analytic wheel term
//            -r * sum I_w * omega_w * (s_hat . y_hat)  -> within +-2 %.
//
// V3 measurement (fixed here, before any off/full run exists):
//   heading frame of the body yaw psi: e_y = (-sin psi, cos psi, 0), e_z = (0, 0, 1).
//   G   = whole-vehicle CG (strut: sprung body mass m - sum m_u at state.position and
//         m_u = max(1, unsprung_mass[i]) at state.unsprung_pos[i]; penalty: state.position).
//   p_i = wheel force point (strut: state.unsprung_pos[i]; penalty: position + R * r_i,
//         r_i = (+-a or -b, +-track/2, -(cg_height - R_w)) as the core mounts it).
//   M   = sum_i ((p_i - G).e_y) Fz_i  -  (mean_i (p_i - G).e_z) * m * V * r
//         (tire normal loads Fz_i plus the total centripetal tire force at the mean
//          wheel-centre height; aero off so no other external force has a roll arm).
//   a   = V * r  (V horizontal speed, r world yaw rate).
//   dM  = mean(M)_full - mean(M)_off * mean(a)_full / mean(a)_off.
//   G_w = mean over the full run of  r_b * sum_i I_w,i * omega_w,i * s_y,i,
//         s_y,i = 1 / sqrt(1 + delta_i^2), delta = front steer command (ackerman 0), rear 0;
//         I_w as the core derives it (wheel_inertia or 0.5 m_u R_w^2).
//   Gate: | dM / (-G_w) - 1 | <= 0.02 on both L5 paths.
//   Precondition (both runs): last 2 s max |V - 7| <= 0.05 m/s, |V/r - 8.5| / 8.5 <= 1 %,
//   std(r) / mean(r) <= 0.5 %.
//
// V3b (added after the V3 measurement, PO verdict on the V3 strut stop): exact angular
// momentum budget about the whole-vehicle CG, heading-frame x axis, same skidpad, same
// off/full a_y normalisation, same gate +-2 % against -G_w, both L5 paths.
//   e_x = (cos psi, sin psi, 0), omega_w = R omega, a_k = omega_w x v_k (steady turn).
//   E = sum_i (p_i - G) x F_i,  F_i = R F_tire,i (penalty)
//                               F_i = m_u a_u,i + R F_tire,i + m_u g e_z (strut particle)
//   T = (x_b - G) x m_b a_b + sum_i (strut) (x_u,i - G) x m_u a_u,i
//   B = omega_w x (R I_b omega)                 (sprung body spin)
//   L = sum_i (strut) (x_b + R r_i - x_u,i) x R F_tire,i  (mount-to-particle couple of
//       the strut connection; 0 on the penalty path)
//   D = (E + L - T - B) . e_x
//   dD = mean(D)_full - mean(D)_off * mean(a)_full / mean(a)_off
//   Gate: | dD / (-G_w) - 1 | <= 0.02, preconditions as V3 with the condition's V and R.
//   Condition 1: R 8.5 m, V 7 m/s (the V3 skidpad; seen in the stop diagnostic).
//   Condition 2: R 15 m, V 9.5 m/s (a_y ~ 6.0 m/s2), fixed here before any run of it.
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
constexpr double kDt = 1e-3;

VehicleParams sedan() {
    VehicleParams vp = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    return vp;
}

TireParams tire() {
    return TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
}

SolverParams l5(bool strut) {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    sp.l5_spatial_suspension = strut;
    return sp;
}

std::string kin(const char* name) {
    return kSrc + "/configs/parts/susp_kinematics/kin/" + name + ".yaml";
}

/// path 0 = L5 penalty, 1 = L5 strut, 2 = L5 strut + corner model (mp sedan / ta sedan).
std::unique_ptr<SimSession> make(const VehicleParams& vp, int path, double v0) {
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    auto s = std::make_unique<SimSession>(create_stunt_dof(), create_flat_ground(0.0, 1.0), vp,
                                          TireSetup(tire()), l5(path > 0), cfg);
    if (path == 2) {
        free_3d_attach_multibody(s->dynamics(), true,
                                 mb::SuspensionTopology::from_yaml(kin("mp_front_sedan")), true);
        free_3d_attach_multibody(s->dynamics(), false,
                                 mb::SuspensionTopology::from_yaml(kin("ta_rear_sedan")), true);
    }
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = v0;
    const double w = v0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    s->reset_settled(s0);
    return s;
}

CmdL4 manoeuvre(int k) {
    CmdL4 u;
    u.throttle = k < 700 ? 0.25 : 0.0;
    u.brake = k >= 1000 ? 0.3 : 0.0;
    u.steer_angle_wheel = k < 200 ? 0.0 : (k < 600 ? 0.05 : -0.03);
    return u;
}

/// Bytes of the state and the per-tick outputs over the whole manoeuvre.
std::vector<double> trace(const VehicleParams& vp, int path, int ticks) {
    auto s = make(vp, path, 12.0);
    std::vector<double> v;
    for (int k = 0; k < ticks; ++k) {
        s->set_input(manoeuvre(k));
        s->tick(kDt);
        const auto o = s->output();
        const State& st = o.state;
        for (int i = 0; i < 3; ++i) v.push_back(st.position[i]);
        for (int i = 0; i < 4; ++i) v.push_back(st.orientation.coeffs()[i]);
        for (int i = 0; i < 3; ++i) v.push_back(st.velocity[i]);
        for (int i = 0; i < 3; ++i) v.push_back(st.angular_velocity[i]);
        for (double x : st.wheel_spin) v.push_back(x);
        for (double x : st.susp_compression) v.push_back(x);
        for (const auto& p : st.unsprung_pos) for (int i = 0; i < 3; ++i) v.push_back(p[i]);
        for (double x : o.Fz) v.push_back(x);
        v.push_back(o.ax);
        v.push_back(o.ay);
        v.push_back(o.roll);
    }
    return v;
}

bool same_bits(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() &&
           std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

VehicleParams with_gyro(const char* mode) {
    VehicleParams vp = sedan();
    vp.wheel_gyro = mode;
    return vp;
}

// ---- V3 skidpad ----------------------------------------------------------------------

struct Skid {
    double M {0}, a {0}, Gw {0}, V {0}, r {0};
    double E {0}, T {0}, B {0}, L {0};   // V3b budget terms, heading-frame x [N m]
    double max_dV {0}, R_rel {0}, r_cv {0};
};

Skid skidpad(const char* mode, bool strut, double kV = 7.0, double kR = 8.5) {
    constexpr double kT = 15.0, kAvg = 2.0;
    VehicleParams vp = with_gyro(mode);
    vp.aero_drag_coeff = 0.0;
    vp.aero_lift_front = 0.0;
    vp.aero_lift_rear = 0.0;
    vp.ackerman_percent = 0.0;
    auto s = make(vp, strut ? 1 : 0, kV);

    const double R_w = vp.wheel_radius_nominal;
    std::array<double, NUM_WHEELS> I_w {}, m_u {};
    double m_u_sum = 0.0;
    for (int i = 0; i < NUM_WHEELS; ++i) {
        const double m_w = vp.unsprung_mass[i] > 0.0 ? vp.unsprung_mass[i] : 25.0;
        I_w[i] = vp.wheel_inertia[i] > 0.0 ? vp.wheel_inertia[i]
                                           : std::max(0.01, 0.5 * m_w * R_w * R_w);
        m_u[i] = std::max(1.0, vp.unsprung_mass[i]);
        m_u_sum += m_u[i];
    }
    const double m = vp.mass;
    const double m_b = strut ? m - m_u_sum : m;
    const double hz = -(vp.cg_height - R_w);
    const std::array<Vec3, NUM_WHEELS> r_mount {{
        Vec3(vp.cg_to_front, 0.5 * vp.track_front, hz),
        Vec3(vp.cg_to_front, -0.5 * vp.track_front, hz),
        Vec3(-vp.cg_to_rear, 0.5 * vp.track_rear, hz),
        Vec3(-vp.cg_to_rear, -0.5 * vp.track_rear, hz)}};

    const int n_tot = static_cast<int>(kT / kDt);
    const int n_avg = static_cast<int>(kAvg / kDt);
    double ie_k = 0.0, ie_v = 0.0;
    double steer = vp.wheelbase / kR;
    std::vector<double> Vs, rs;
    Skid out;
    for (int k = 0; k < n_tot; ++k) {
        const auto o = s->output();
        const State& st = o.state;
        const Mat3 Rm = st.orientation.toRotationMatrix();
        const Vec3 v_w = Rm * st.velocity;
        const double V = std::hypot(v_w.x(), v_w.y());
        const double r = (Rm * st.angular_velocity).z();
        // curvature + speed PI hold (deterministic, same gains for every run)
        const double e_k = 1.0 / kR - r / std::max(V, 0.5);
        const double e_v = kV - V;
        ie_k += e_k * kDt;
        ie_v += e_v * kDt;
        steer = std::clamp(vp.wheelbase / kR + 1.0 * e_k + 2.0 * ie_k, -0.45, 0.45);
        CmdL4 u;
        u.steer_angle_wheel = steer;
        u.throttle = std::clamp(0.1 + 0.5 * e_v + 0.3 * ie_v, 0.0, 1.0);
        u.brake = 0.0;

        if (k >= n_tot - n_avg) {
            const double psi = std::atan2(Rm(1, 0), Rm(0, 0));
            const Vec3 e_y(-std::sin(psi), std::cos(psi), 0.0);
            std::array<Vec3, NUM_WHEELS> p {};
            Vec3 G = m_b * st.position;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                p[i] = strut ? st.unsprung_pos[i] : Vec3(st.position + Rm * r_mount[i]);
                if (strut) G += m_u[i] * st.unsprung_pos[i];
            }
            G /= (strut ? m : m_b);
            double M = 0.0, z_mean = 0.0;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                M += (p[i] - G).dot(e_y) * o.Fz[i];
                z_mean += 0.25 * (p[i].z() - G.z());
            }
            M -= z_mean * m * V * r;
            double H_y = 0.0;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const double d = i < 2 ? steer : 0.0;
                H_y += I_w[i] * st.wheel_spin[i] / std::sqrt(1.0 + d * d);
            }
            // V3b budget (exact angular momentum about G, heading-frame x)
            const Vec3 e_x(std::cos(psi), std::sin(psi), 0.0);
            const Vec3 w_w = Rm * st.angular_velocity;
            Vec3 E = Vec3::Zero(), T = Vec3::Zero(), L = Vec3::Zero();
            T += (st.position - G).cross(m_b * w_w.cross(v_w));
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const Vec3 F_t = Rm * o.tire_forces[i];
                Vec3 F = F_t;
                if (strut) {
                    const Vec3 a_u = w_w.cross(st.unsprung_vel[i]);
                    F += m_u[i] * a_u + Vec3(0.0, 0.0, m_u[i] * 9.80665);
                    T += (st.unsprung_pos[i] - G).cross(m_u[i] * a_u);
                    L += (st.position + Rm * r_mount[i] - st.unsprung_pos[i]).cross(F_t);
                }
                E += (p[i] - G).cross(F);
            }
            const Vec3& Ib = vp.inertia_diag;
            const Vec3 H_b = Rm * Vec3(Ib.x() * st.angular_velocity.x(),
                                       Ib.y() * st.angular_velocity.y(),
                                       Ib.z() * st.angular_velocity.z());
            out.E += E.dot(e_x) / n_avg;
            out.T += T.dot(e_x) / n_avg;
            out.B += w_w.cross(H_b).dot(e_x) / n_avg;
            out.L += L.dot(e_x) / n_avg;
            out.M += M / n_avg;
            out.a += V * r / n_avg;
            out.Gw += st.angular_velocity.z() * H_y / n_avg;
            out.V += V / n_avg;
            out.r += r / n_avg;
            Vs.push_back(V);
            rs.push_back(r);
        }
        s->set_input(u);
        s->tick(kDt);
    }
    double var = 0.0;
    for (std::size_t j = 0; j < Vs.size(); ++j) {
        out.max_dV = std::max(out.max_dV, std::abs(Vs[j] - kV));
        var += (rs[j] - out.r) * (rs[j] - out.r) / rs.size();
    }
    out.R_rel = std::abs(out.V / out.r - kR) / kR;
    out.r_cv = std::sqrt(var) / std::abs(out.r);
    return out;
}

void v3(bool strut) {
    const Skid off = skidpad("off", strut);
    const Skid full = skidpad("full", strut);
    for (const auto* run : {&off, &full}) {
        std::printf("V3 %s %s V=%.6f r=%.6f R=%.6f max|dV|=%.3e R_rel=%.3e r_cv=%.3e "
                    "M=%.6f N m a=%.6f m/s2 Gw=%.6f N m\n",
                    strut ? "strut" : "penalty", run == &off ? "off " : "full", run->V,
                    run->r, run->V / run->r, run->max_dV, run->R_rel, run->r_cv, run->M,
                    run->a, run->Gw);
        EXPECT_LE(run->max_dV, 0.05) << "precondition: speed hold";
        EXPECT_LE(run->R_rel, 0.01) << "precondition: radius hold";
        EXPECT_LE(run->r_cv, 0.005) << "precondition: steady yaw rate";
    }
    const double dM = full.M - off.M * full.a / off.a;
    const double expected = -full.Gw;
    const double rel = dM / expected - 1.0;
    std::printf("V3 %s dM=%.6f N m expected=%.6f N m rel=%+.4f %% (gate +-2 %%)\n",
                strut ? "strut" : "penalty", dM, expected, 100.0 * rel);
    EXPECT_LE(std::abs(rel), 0.02);
}

void v3b(bool strut, double V, double R) {
    const Skid off = skidpad("off", strut, V, R);
    const Skid full = skidpad("full", strut, V, R);
    const char* path = strut ? "strut" : "penalty";
    for (const auto* run : {&off, &full}) {
        std::printf("V3b %s R=%.1f V=%.1f %s V=%.6f r=%.6f R=%.6f max|dV|=%.3e R_rel=%.3e "
                    "r_cv=%.3e E=%.6f T=%.6f B=%.6f L=%.6f N m a=%.6f m/s2 Gw=%.6f N m\n",
                    path, R, V, run == &off ? "off " : "full", run->V, run->r, run->V / run->r,
                    run->max_dV, run->R_rel, run->r_cv, run->E, run->T, run->B, run->L, run->a,
                    run->Gw);
        EXPECT_LE(run->max_dV, 0.05) << "precondition: speed hold";
        EXPECT_LE(run->R_rel, 0.01) << "precondition: radius hold";
        EXPECT_LE(run->r_cv, 0.005) << "precondition: steady yaw rate";
    }
    const auto D = [](const Skid& x) { return x.E + x.L - x.T - x.B; };
    const double dD = D(full) - D(off) * full.a / off.a;
    const double expected = -full.Gw;
    const double rel = dD / expected - 1.0;
    std::printf("V3b %s R=%.1f V=%.1f dD=%.6f N m expected=%.6f N m rel=%+.4f %% (gate +-2 %%)\n",
                path, R, V, dD, expected, 100.0 * rel);
    EXPECT_LE(std::abs(rel), 0.02);
}

}  // namespace

// ---- V2 (sim level) --------------------------------------------------------------------

TEST(L5WheelGyro, V2_UnsetLegacyFullBitIdenticalAtZeroCamber) {
    for (int path : {0, 1}) {
        const auto unset = trace(sedan(), path, 1500);
        EXPECT_TRUE(same_bits(unset, trace(with_gyro("legacy"), path, 1500)))
            << "legacy == unset, path " << path;
        EXPECT_TRUE(same_bits(unset, trace(with_gyro("full"), path, 1500)))
            << "full == legacy at zero camber, path " << path;
        EXPECT_FALSE(same_bits(unset, trace(with_gyro("off"), path, 1500)))
            << "off removes the term, path " << path;
    }
}

TEST(L5WheelGyro, V2_CornerModelUnsetLegacyAndFullReactsToCamber) {
    const auto unset = trace(sedan(), 2, 600);
    EXPECT_TRUE(same_bits(unset, trace(with_gyro("legacy"), 2, 600))) << "legacy == unset (DAE)";
    const auto full = trace(with_gyro("full"), 2, 600);
    EXPECT_FALSE(same_bits(unset, full)) << "full must see the DAE camber";
    auto s = make(sedan(), 2, 12.0);
    const auto g = free_3d_wheel_camber(s->dynamics());
    std::printf("V2-info DAE sedan static camber [deg] FL=%.4f FR=%.4f RL=%.4f RR=%.4f\n",
                g[0] * 57.29577951308232, g[1] * 57.29577951308232,
                g[2] * 57.29577951308232, g[3] * 57.29577951308232);
}

// ---- V3 -------------------------------------------------------------------------------

TEST(L5WheelGyro, V3_SkidpadRollMomentPenalty) { v3(false); }

TEST(L5WheelGyro, V3_SkidpadRollMomentStrut) { v3(true); }

// ---- V3b (exact budget incl. the strut mount couple) ----------------------------------

TEST(L5WheelGyro, V3b_ExactBudgetPenaltyR8p5V7) { v3b(false, 7.0, 8.5); }

TEST(L5WheelGyro, V3b_ExactBudgetStrutR8p5V7) { v3b(true, 7.0, 8.5); }

TEST(L5WheelGyro, V3b_ExactBudgetPenaltyR15V9p5) { v3b(false, 9.5, 15.0); }

TEST(L5WheelGyro, V3b_ExactBudgetStrutR15V9p5) { v3b(true, 9.5, 15.0); }
