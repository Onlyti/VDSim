// Work order 49a (L5 strut + corner-model DAE: contact-patch jacking + prescribed travel
// acceleration), integration gates fixed before the implementation commits.
//
// Common setup (the stage-3 diagnosis probe, unchanged): generic sedan, aero off (elements
// cleared, drag 0, lifts 0), L5 stunt + l5_spatial_suspension, max_substep_dt 2e-4 s,
// max_substeps 64, nominal_dt 1e-3 s, generic_pacejka tyre, flat ground mu 1, front + rear
// corner models attached with the DAE on, spawn position.z = cg_height, wheel spin v0 / R,
// reset_settled.
//   antidive = tests/fixtures/dw_front_antidive_sample + ta_rear_sedan
//   sports   = dw_front_sports + 5link_rear_sports
//
// P1 : zero command (coast), 10 s = 10000 ticks, v0 25 and 35 m/s, both fixtures.
//      q = state.angular_velocity.y() after tick k (t = (k+1) 1e-3 s).
//      q_rms  = RMS of q about its mean over ticks k = 4999..9998  (t in [5, 10) s)
//      growth = RMS(q, k = 7999..9998) / RMS(q, k = 1999..3998)   (t in [8,10) / [2,4) s)
//      Gate: q_rms <= 1e-3 rad/s and growth < 0.5, all four cases.
//      Same windows as the diagnosis analysis script (analyze3.py, sha256 4c9a1a108ce24ef8),
//      written here with integer tick indices.
// P2 : the same probe at v0 10 m/s, both fixtures: q_rms <= the pre-fix value of the main
//      this branch is merged onto (PRs #37 and #38 merged), same window. The pre-fix values
//      are recorded in kP2PreFix by a separate commit on the merged main before the
//      implementation commits; until then the test fails ("not recorded").
// P10: reaction consistency. Steady circle R 8.5 m, V 7 m/s (the work order 50 V3/V3b
//      skidpad: ackerman 0, curvature + speed PI hold with the same gains, 15 s, last 2 s
//      mean), both fixtures. Exact angular momentum budget about the whole-vehicle CG G,
//      heading-frame x axis e_x = (cos psi, sin psi, 0), steady accelerations a = omega_w x v:
//        m_u,i = max(1, unsprung_mass[i]), m_b = m - sum m_u, G = (m_b x_b + sum m_u x_u,i)/m
//        h_s   = (m cg_height - sum m_u R) / m_b   (whole-vehicle cg_height, work order 47)
//        x_m,i = x_b + R_wb (+a or -b, +-track/2, -(h_s - R))  strut mount on the body
//        F_t,i = R_wb tire_forces[i]  (tyre force on the wheel particle)
//        E = sum (x_u,i - G) x F_t,i
//        L = sum (x_m,i - x_u,i) x (F_t,i - m_u a_u,i - m_u g e_z)
//            (moment of the strut connection pair: force on the particle at x_u, its
//             reaction on the body at the mount; g = 9.80665)
//        T = (x_b - G) x m_b a_b + sum (x_u,i - G) x m_u a_u,i
//        B = omega_w x (R_wb I_b omega_b)
//        W = -Gw,  Gw = r_b sum I_w,i omega_w,i s_y,i  (wheel spin precession, as V3:
//            s_y = 1 / sqrt(1 + delta^2), delta = front steer command, rear 0)
//        D = (E + L - T - B) . e_x - W
//        S = m |a_y| cg_height,  a_y = V r
//      Gate: |mean D| <= 0.02 mean S on both fixtures, V3 preconditions (last 2 s
//      max |V - 7| <= 0.05 m/s, |V/r - 8.5| / 8.5 <= 1 %, std(r)/mean(r) <= 0.5 %).
//      Not in D: torques along the wheel spin axes (drive/brake and tyre rolling reaction);
//      their roll part is bounded by sum |Fx,i| R |sin delta_i| and printed (information).
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
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
constexpr double kDt = 1e-3;
constexpr double kG = 9.80665;

/// P2 pre-fix q_rms [rad/s] at 10 m/s on the merged main: {antidive, sports}. NaN = not yet
/// recorded (filled by the baseline commit before the implementation commits).
constexpr std::array<double, 2> kP2PreFix {{std::numeric_limits<double>::quiet_NaN(),
                                            std::numeric_limits<double>::quiet_NaN()}};

/**
 * @brief Generic sedan with aero off, as the diagnosis probe.
 * @return vehicle parameters
 */
VehicleParams sedan() {
    VehicleParams vp = VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
    vp.aero_elements.clear();
    vp.aero_drag_coeff = 0.0;
    vp.aero_lift_front = 0.0;
    vp.aero_lift_rear = 0.0;
    return vp;
}

/**
 * @brief L5 strut + corner-model session for a fixture at an initial speed.
 * @param vp      vehicle parameters
 * @param fixture 0 = antidive, 1 = sports
 * @param v0      initial forward speed [m/s]
 * @return settled session
 */
std::unique_ptr<SimSession> make(const VehicleParams& vp, int fixture, double v0) {
    SolverParams sp;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 64;
    sp.stunt_physics = true;
    sp.l5_spatial_suspension = true;
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    auto tp = TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
    auto s = std::make_unique<SimSession>(create_stunt_dof(), create_flat_ground(0.0, 1.0), vp,
                                          TireSetup(tp), sp, cfg);
    const std::string kd = kSrc + "/configs/parts/susp_kinematics/kin/";
    const std::string ff = fixture == 0 ? kSrc + "/tests/fixtures/dw_front_antidive_sample.yaml"
                                        : kd + "dw_front_sports.yaml";
    const std::string rr = fixture == 0 ? kd + "ta_rear_sedan.yaml" : kd + "5link_rear_sports.yaml";
    EXPECT_TRUE(free_3d_attach_multibody(s->dynamics(), true,
                                         mb::SuspensionTopology::from_yaml(ff), true));
    EXPECT_TRUE(free_3d_attach_multibody(s->dynamics(), false,
                                         mb::SuspensionTopology::from_yaml(rr), true));
    State s0;
    s0.position.z() = vp.cg_height;
    s0.velocity.x() = v0;
    const double w = v0 / vp.wheel_radius_nominal;
    s0.wheel_spin = {{w, w, w, w}};
    s->reset_settled(s0);
    return s;
}

/**
 * @brief Fixture label.
 * @param fixture 0 = antidive, 1 = sports
 * @return name
 */
const char* name(int fixture) { return fixture == 0 ? "antidive" : "sports"; }

struct Pitch {
    double q_rms {0}, growth {0};
};

/**
 * @brief RMS about the mean of q over ticks [k0, k1].
 * @param q  per-tick pitch rate [rad/s]
 * @param k0 first tick index
 * @param k1 last tick index (inclusive)
 * @return RMS [rad/s]
 */
double rms(const std::vector<double>& q, int k0, int k1) {
    double mean = 0.0;
    for (int k = k0; k <= k1; ++k) mean += q[k];
    mean /= (k1 - k0 + 1);
    double acc = 0.0;
    for (int k = k0; k <= k1; ++k) acc += (q[k] - mean) * (q[k] - mean);
    return std::sqrt(acc / (k1 - k0 + 1));
}

/**
 * @brief Coast probe (zero command, 10 s) and the P1 window metrics.
 * @param fixture 0 = antidive, 1 = sports
 * @param v0      initial speed [m/s]
 * @return q_rms (t in [5,10) s) and growth ([8,10) / [2,4) s)
 */
Pitch coast(int fixture, double v0) {
    const VehicleParams vp = sedan();
    auto s = make(vp, fixture, v0);
    std::vector<double> q(10000, 0.0);
    const CmdL4 u;
    for (int k = 0; k < 10000; ++k) {
        s->set_input(u);
        s->tick(kDt);
        q[k] = s->output().state.angular_velocity.y();
    }
    Pitch p;
    p.q_rms = rms(q, 4999, 9998);
    p.growth = rms(q, 7999, 9998) / std::max(rms(q, 1999, 3998), 1e-15);
    std::printf("P1-probe %s v0=%.0f q_rms=%.4e rad/s growth=%.4f\n", name(fixture), v0,
                p.q_rms, p.growth);
    return p;
}

/**
 * @brief P1 gate for one fixture and speed.
 * @param fixture 0 = antidive, 1 = sports
 * @param v0      initial speed [m/s]
 */
void p1(int fixture, double v0) {
    const Pitch p = coast(fixture, v0);
    EXPECT_LE(p.q_rms, 1e-3) << name(fixture) << " v0 " << v0;
    EXPECT_LT(p.growth, 0.5) << name(fixture) << " v0 " << v0;
}

/**
 * @brief P2 gate: 10 m/s q_rms not above the recorded pre-fix value.
 * @param fixture 0 = antidive, 1 = sports
 */
void p2(int fixture) {
    const Pitch p = coast(fixture, 10.0);
    const double base = kP2PreFix[fixture];
    ASSERT_FALSE(std::isnan(base)) << "P2 pre-fix baseline not recorded for " << name(fixture);
    std::printf("P2 %s q_rms=%.4e pre-fix=%.4e rad/s\n", name(fixture), p.q_rms, base);
    EXPECT_LE(p.q_rms, base) << name(fixture);
}

struct Budget {
    double D {0}, S {0}, spin_bound {0}, V {0}, r {0};
    double max_dV {0}, R_rel {0}, r_cv {0};
};

/**
 * @brief P10 steady-circle exact roll budget about the whole-vehicle CG.
 * @param fixture 0 = antidive, 1 = sports
 * @return mean residual D, scale S, spin-axis torque bound and the preconditions
 */
Budget skidpad(int fixture) {
    constexpr double kV = 7.0, kRad = 8.5, kT = 15.0, kAvg = 2.0;
    VehicleParams vp = sedan();
    vp.ackerman_percent = 0.0;
    auto s = make(vp, fixture, kV);

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
    const double m_b = m - m_u_sum;
    const double h_s = (m * vp.cg_height - m_u_sum * R_w) / m_b;
    const double hz = -(h_s - R_w);
    const std::array<Vec3, NUM_WHEELS> r_mount {{
        Vec3(vp.cg_to_front, 0.5 * vp.track_front, hz),
        Vec3(vp.cg_to_front, -0.5 * vp.track_front, hz),
        Vec3(-vp.cg_to_rear, 0.5 * vp.track_rear, hz),
        Vec3(-vp.cg_to_rear, -0.5 * vp.track_rear, hz)}};

    const int n_tot = static_cast<int>(kT / kDt);
    const int n_avg = static_cast<int>(kAvg / kDt);
    double ie_k = 0.0, ie_v = 0.0;
    double steer = vp.wheelbase / kRad;
    std::vector<double> Vs, rs;
    Budget out;
    for (int k = 0; k < n_tot; ++k) {
        const auto o = s->output();
        const State& st = o.state;
        const Mat3 Rm = st.orientation.toRotationMatrix();
        const Vec3 v_w = Rm * st.velocity;
        const double V = std::hypot(v_w.x(), v_w.y());
        const double r = (Rm * st.angular_velocity).z();
        const double e_k = 1.0 / kRad - r / std::max(V, 0.5);
        const double e_v = kV - V;
        ie_k += e_k * kDt;
        ie_v += e_v * kDt;
        steer = std::clamp(vp.wheelbase / kRad + 1.0 * e_k + 2.0 * ie_k, -0.45, 0.45);
        CmdL4 u;
        u.steer_angle_wheel = steer;
        u.throttle = std::clamp(0.1 + 0.5 * e_v + 0.3 * ie_v, 0.0, 1.0);
        u.brake = 0.0;

        if (k >= n_tot - n_avg) {
            const double psi = std::atan2(Rm(1, 0), Rm(0, 0));
            const Vec3 e_x(std::cos(psi), std::sin(psi), 0.0);
            const Vec3 w_w = Rm * st.angular_velocity;
            Vec3 G = m_b * st.position;
            for (int i = 0; i < NUM_WHEELS; ++i) G += m_u[i] * st.unsprung_pos[i];
            G /= m;
            Vec3 E = Vec3::Zero(), L = Vec3::Zero();
            Vec3 T = (st.position - G).cross(m_b * w_w.cross(v_w));
            double H_y = 0.0, spin = 0.0;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                const Vec3& x_u = st.unsprung_pos[i];
                const Vec3 x_m = st.position + Rm * r_mount[i];
                const Vec3 F_t = Rm * o.tire_forces[i];
                const Vec3 a_u = w_w.cross(st.unsprung_vel[i]);
                E += (x_u - G).cross(F_t);
                L += (x_m - x_u).cross(F_t - m_u[i] * a_u - Vec3(0.0, 0.0, m_u[i] * kG));
                T += (x_u - G).cross(m_u[i] * a_u);
                const double d = i < 2 ? steer : 0.0;
                H_y += I_w[i] * st.wheel_spin[i] / std::sqrt(1.0 + d * d);
                spin += std::abs(o.tire_forces[i].x()) * R_w * std::abs(std::sin(d));
            }
            const Vec3& Ib = vp.inertia_diag;
            const Vec3 H_b = Rm * Vec3(Ib.x() * st.angular_velocity.x(),
                                       Ib.y() * st.angular_velocity.y(),
                                       Ib.z() * st.angular_velocity.z());
            const double Gw = st.angular_velocity.z() * H_y;
            const double D = (E + L - T - w_w.cross(H_b)).dot(e_x) + Gw;
            out.D += D / n_avg;
            out.S += m * std::abs(V * r) * vp.cg_height / n_avg;
            out.spin_bound += spin / n_avg;
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
    out.R_rel = std::abs(out.V / out.r - kRad) / kRad;
    out.r_cv = std::sqrt(var) / std::abs(out.r);
    return out;
}

/**
 * @brief P10 gate for one fixture.
 * @param fixture 0 = antidive, 1 = sports
 */
void p10(int fixture) {
    const Budget b = skidpad(fixture);
    const double rel = b.D / b.S;
    std::printf("P10 %s V=%.6f r=%.6f max|dV|=%.3e R_rel=%.3e r_cv=%.3e D=%.6f N m "
                "S=%.6f N m D/S=%+.4f %% (gate +-2 %%) spin-axis bound=%.3f N m (info)\n",
                name(fixture), b.V, b.r, b.max_dV, b.R_rel, b.r_cv, b.D, b.S, 100.0 * rel,
                b.spin_bound);
    EXPECT_LE(b.max_dV, 0.05) << "precondition: speed hold";
    EXPECT_LE(b.R_rel, 0.01) << "precondition: radius hold";
    EXPECT_LE(b.r_cv, 0.005) << "precondition: steady yaw rate";
    EXPECT_LE(std::abs(rel), 0.02) << name(fixture);
}

}  // namespace

TEST(L5DaePitch, P1_Antidive25) { p1(0, 25.0); }
TEST(L5DaePitch, P1_Antidive35) { p1(0, 35.0); }
TEST(L5DaePitch, P1_Sports25) { p1(1, 25.0); }
TEST(L5DaePitch, P1_Sports35) { p1(1, 35.0); }
TEST(L5DaePitch, P2_Antidive10NotWorse) { p2(0); }
TEST(L5DaePitch, P2_Sports10NotWorse) { p2(1); }
TEST(L5DaePitch, P10_AntidiveSkidpadRollBudget) { p10(0); }
TEST(L5DaePitch, P10_SportsSkidpadRollBudget) { p10(1); }
