// 48b T1–T9 (C++). Locked T5 table: dw_front_sports.yaml, Convention A.
#include "vdsim/control.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <stdlib.h>
#endif

using namespace vdsim;

namespace {

const std::string kSrc = VDSIM_SOURCE_DIR;
constexpr double kDt = 1e-3;
constexpr double kRackM = 0.0225;
constexpr double kTol = 0.02;

// Locked T5 table (product toe [deg], PO 48b phase-1). Do not edit after lock.
constexpr double kT5[2][4] = {
    /* +22.5 mm */ {+4.546251, +4.554464, 0.0, 0.0},
    /* -22.5 mm */ {-4.554464, -4.546251, 0.0, 0.0},
};

/** Test-only physical rack source; CmdL4 keeps its wheel-angle contract. */
class RackOutput final : public ISteeringSystem {
public:
    explicit RackOutput(double rack) : rack_(rack) {}
    SteeringOutput apply(const SubsystemContext&) override {
        SteeringOutput out;
        out.roadwheel_angle = 0.0;
        out.rack_travel = rack_;
        return out;
    }
private:
    double rack_;
};

/** Replace only the approved steering module slot with a physical test input. */
void inject_rack(SimSession& session, double rack) {
    if (!session.dynamics().set_steering_module(std::make_shared<RackOutput>(rack)))
        throw std::runtime_error("L5 steering module replacement rejected");
}

VehicleParams sedan() {
    return VehicleParams::from_yaml(kSrc + "/configs/vehicles/generic_sedan.yaml");
}

TireParams tire() {
    return TireParams::from_yaml(kSrc + "/configs/parts/tire/generic_pacejka.yaml");
}

std::string dw_front_sports() {
    return kSrc + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
}

SolverParams l5_dae() {
    SolverParams sp;
    sp.stunt_physics = true;
    sp.max_substep_dt = 2e-4;
    sp.max_substeps = 16;
    sp.l5_spatial_suspension = true;
    return sp;
}

std::unique_ptr<SimSession> make_l5(const VehicleParams& vp, bool attach) {
    SimConfig cfg;
    cfg.nominal_dt = kDt;
    auto s = std::make_unique<SimSession>(create_stunt_dof(), create_flat_ground(0.0, 1.0), vp,
                                          TireSetup(tire()), l5_dae(), cfg);
    if (attach) {
        free_3d_attach_multibody(s->dynamics(), true,
                                 mb::SuspensionTopology::from_yaml(dw_front_sports()), true);
    }
    State s0;
    s0.position.z() = vp.cg_height;
    s->reset_settled(s0);
    return s;
}

std::vector<double> dump(SimSession& s) {
    const auto o = s.output();
    std::vector<double> v;
    v.push_back(o.state.position.x());
    v.push_back(o.state.position.y());
    v.push_back(o.state.position.z());
    v.push_back(o.state.velocity.x());
    v.push_back(o.state.velocity.y());
    v.push_back(o.state.velocity.z());
    v.push_back(o.state.yaw());
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(o.state.wheel_spin[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(o.Fz[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(o.state.susp_compression[i]);
    return v;
}

void tick_n(SimSession& s, const CmdL4& u, int n) {
    for (int k = 0; k < n; ++k) {
        s.set_input(u);
        s.tick(kDt);
    }
}

std::array<double, NUM_WHEELS> toe_deg(IVehicleDynamics& dyn) {
    const auto t = free_3d_wheel_toe(dyn);
    std::array<double, NUM_WHEELS> d{};
    for (int i = 0; i < NUM_WHEELS; ++i) d[i] = t[i] * 180.0 / 3.14159265358979323846;
    return d;
}


/** Synchronized loaded runs at the first common finite-time quasi-static window.
 * Thresholds and the eight-second deadline are fixed by the work order.
 * This does not assert permanent equilibrium after that window.
 */
struct SettledRackRuns {
    int tick = 0;
    std::array<std::vector<double>, 3> values;
    std::array<std::array<double, NUM_WHEELS>, 3> toe;
};

/** Require every monitored state/load/geometry channel to remain finite. */
bool monitored_finite(const SimOutput& o,
                      const std::array<double, NUM_WHEELS>& toe,
                      const std::array<double, NUM_WHEELS>& camber) {
    const auto& st = o.state;
    if (!st.position.allFinite() || !st.orientation.coeffs().allFinite() ||
        !st.velocity.allFinite() || !st.angular_velocity.allFinite()) return false;
    for (int i = 0; i < NUM_WHEELS; ++i) {
        if (!std::isfinite(st.susp_compression[i]) ||
            !std::isfinite(st.susp_velocity[i]) ||
            !std::isfinite(st.wheel_spin[i]) ||
            !st.unsprung_pos[i].allFinite() || !st.unsprung_vel[i].allFinite() ||
            !std::isfinite(toe[i]) || !std::isfinite(camber[i]) ||
            !std::isfinite(o.Fz[i]) || !o.tire_forces[i].allFinite()) return false;
    }
    return true;
}

/** Advance zero/positive/negative rack together; never search past the first window. */
SettledRackRuns first_common_settlement(double ackerman) {
    const std::array<double, 3> racks{0.0, kRackM, -kRackM};
    std::array<std::unique_ptr<SimSession>, 3> sessions;
    std::array<std::array<double, NUM_WHEELS>, 3> previous_toe{}, previous_fz{};
    for (int r = 0; r < 3; ++r) {
        auto vp = sedan();
        vp.steer_input = "rack_travel";
        vp.ackerman_percent = ackerman;
        sessions[r] = make_l5(vp, true);
        inject_rack(*sessions[r], 0.0);
        previous_toe[r] = toe_deg(sessions[r]->dynamics());
        previous_fz[r] = sessions[r]->output().Fz;
    }
    int common_ticks = 0;
    SettledRackRuns result;
    for (int k = 1; k <= 8000; ++k) {
        bool common_stable = k > 200;
        for (int r = 0; r < 3; ++r) {
            if (k == 201) inject_rack(*sessions[r], racks[r]);
            sessions[r]->set_input(CmdL4{});
            sessions[r]->tick(kDt);
            const auto o = sessions[r]->output();
            const auto toe = toe_deg(sessions[r]->dynamics());
            const auto camber = free_3d_wheel_camber(sessions[r]->dynamics());
            if (!monitored_finite(o, toe, camber))
                throw std::runtime_error("non-finite loaded rack run");
            bool stable = o.state.velocity.norm() <= 0.001 &&
                          o.state.angular_velocity.norm() <= 0.001;
            for (int i = 0; i < NUM_WHEELS; ++i) {
                if (!(o.Fz[i] > 0.0))
                    throw std::runtime_error("non-positive loaded rack wheel force");
                stable = stable && std::abs(o.state.susp_velocity[i]) <= 0.001 &&
                    std::abs(toe[i] - previous_toe[r][i]) / kDt <= 0.01 &&
                    std::abs(o.Fz[i] - previous_fz[r][i]) / kDt <= 1.0;
            }
            common_stable = common_stable && stable;
            previous_toe[r] = toe;
            previous_fz[r] = o.Fz;
            result.toe[r] = toe;
        }
        common_ticks = common_stable ? common_ticks + 1 : 0;
        if (common_ticks >= 500) {
            result.tick = k;
            for (int r = 0; r < 3; ++r) {
                result.values[r] = dump(*sessions[r]);
                for (double x : result.toe[r]) result.values[r].push_back(x);
            }
            std::printf("first_common_settlement t=%.3f s held=%d dt=%.3f ack=%.0f\n",
                        k * kDt, common_ticks, kDt, ackerman);
            return result;
        }
    }
    throw std::runtime_error("no common 0.5 s settlement within 8 s");
}

/** Compare same-time differences with the original locked table and tolerances. */
bool locked_geometry_matches(const SettledRackRuns& result) {
    bool matches = true;
    for (int sign = 0; sign < 2; ++sign) {
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double delta = result.toe[sign + 1][i] - result.toe[0][i];
            const double expected = kT5[sign][i];
            const double band = i < 2 ? std::abs(expected) * kTol : 1e-6;
            matches = matches && std::abs(delta - expected) <= band;
        }
    }
    return matches;
}

void set_disable_env(bool on) {
#ifdef _WIN32
    _putenv_s("VDSIM_DISABLE_L5_RACK_TRAVEL", on ? "1" : "");
#else
    if (on) setenv("VDSIM_DISABLE_L5_RACK_TRAVEL", "1", 1);
    else unsetenv("VDSIM_DISABLE_L5_RACK_TRAVEL");
#endif
}

}  // namespace

TEST(L5RackTravel, T1T2_DefaultMatchesRoadwheel) {
    VehicleParams a = sedan();
    VehicleParams b = sedan();
    b.steer_input = "roadwheel";
    auto sa = make_l5(a, true);
    auto sb = make_l5(b, true);
    CmdL4 u;
    u.steer_angle_wheel = 0.05;
    tick_n(*sa, u, 80);
    tick_n(*sb, u, 80);
    const auto da = dump(*sa);
    const auto db = dump(*sb);
    ASSERT_EQ(da.size(), db.size());
    EXPECT_EQ(0, std::memcmp(da.data(), db.data(), da.size() * sizeof(double)));
}

TEST(L5RackTravel, T3_RackWithoutHardpointThrows) {
    VehicleParams vp = sedan();
    vp.steer_input = "rack_travel";
    auto s = make_l5(vp, false);
    CmdL4 u;
    u.steer_angle_wheel = kRackM;
    EXPECT_THROW(
        {
            s->set_input(u);
            s->tick(kDt);
        },
        std::invalid_argument);
}

TEST(L5RackTravel, T4_RackPlusDynamicThrows) {
    VehicleParams vp = sedan();
    vp.steer_input = "rack_travel";
    vp.steering_dynamic = true;
    EXPECT_THROW(make_l5(vp, false), std::invalid_argument);
}

TEST(L5RackTravel, T7_BadKeyThrows) {
    VehicleParams vp = sedan();
    vp.steer_input = "nope";
    EXPECT_THROW(make_l5(vp, false), std::invalid_argument);
}

TEST(L5RackTravel, T5T6T8_LockedTableAndSameSign) {
    const auto settled = first_common_settlement(0.0);
    for (int sign = 0; sign < 2; ++sign) {
        for (int i = 0; i < NUM_WHEELS; ++i) {
            const double delta = settled.toe[sign + 1][i] - settled.toe[0][i];
            const double expected = kT5[sign][i];
            const double band = i < 2 ? std::abs(expected) * kTol : 1e-6;
            std::printf("T5 sign=%d wheel=%d delta=%.12f expected=%.6f band=%.9f\n",
                        sign, i, delta, expected, band);
            EXPECT_NEAR(delta, expected, band) << "T5 sign/wheel " << sign << '/' << i;
            if (i < 2) {
                EXPECT_GT(delta * expected, 0.0) << "T6 both input signs";
            }
        }
    }
}

/** Independent both-sign Ackerman byte comparison at common settled observations. */
TEST(L5RackTravel, T8_AckermanDoesNotChangeRackOutput) {
    const auto no_ackerman = first_common_settlement(0.0);
    const auto full_ackerman = first_common_settlement(100.0);
    ASSERT_EQ(no_ackerman.tick, full_ackerman.tick);
    for (int r = 0; r < 3; ++r) {
        const auto& a = no_ackerman.values[r];
        const auto& b = full_ackerman.values[r];
        ASSERT_EQ(a.size(), b.size());
        EXPECT_EQ(0, std::memcmp(a.data(), b.data(), a.size() * sizeof(double)))
            << "T8 zero/positive/negative run " << r;
    }
}

/** The removed direct path must miss the same formal settled table; restore must pass. */
TEST(L5RackTravel, T9_DisableEnvFailsT5ThenRestores) {
    struct RestoreDirectPath {
        ~RestoreDirectPath() { set_disable_env(false); }
    } restore;
    set_disable_env(true);
    const auto disabled = first_common_settlement(0.0);
    EXPECT_FALSE(locked_geometry_matches(disabled));
    set_disable_env(false);
    const auto enabled = first_common_settlement(0.0);
    EXPECT_TRUE(locked_geometry_matches(enabled));
}
