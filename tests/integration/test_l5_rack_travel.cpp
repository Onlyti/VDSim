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
    VehicleParams vp = sedan();
    vp.steer_input = "rack_travel";
    auto s0 = make_l5(vp, true);
    inject_rack(*s0, 0.0);
    tick_n(*s0, CmdL4{}, 300);
    const auto base = toe_deg(s0->dynamics());

    auto run_rack = [&](double rack, double ack) {
        VehicleParams v = sedan();
        v.steer_input = "rack_travel";
        v.ackerman_percent = ack;
        auto s = make_l5(v, true);
        tick_n(*s, CmdL4{}, 200);
        CmdL4 u;
        inject_rack(*s, rack);
        tick_n(*s, u, 100);
        auto t = toe_deg(s->dynamics());
        for (int i = 0; i < NUM_WHEELS; ++i) t[i] -= base[i];
        return t;
    };

    const auto plus = run_rack(+kRackM, 0.0);
    const auto minus = run_rack(-kRackM, 0.0);
    const auto plus_ack = run_rack(+kRackM, 100.0);

    std::printf("T5 +22.5 dtoe deg FL=%.6f FR=%.6f RL=%.6f RR=%.6f\n",
                plus[0], plus[1], plus[2], plus[3]);
    std::printf("T5 -22.5 dtoe deg FL=%.6f FR=%.6f RL=%.6f RR=%.6f\n",
                minus[0], minus[1], minus[2], minus[3]);

    for (int i = 0; i < 4; ++i) {
        const double exp_p = kT5[0][i];
        const double exp_m = kT5[1][i];
        const double band_p = std::max(1e-6, std::abs(exp_p) * kTol);
        const double band_m = std::max(1e-6, std::abs(exp_m) * kTol);
        EXPECT_NEAR(plus[i], exp_p, band_p) << "T5 + wheel " << i;
        EXPECT_NEAR(minus[i], exp_m, band_m) << "T5 - wheel " << i;
        EXPECT_NEAR(plus_ack[i], plus[i], 1e-12) << "T8 ackerman unused wheel " << i;
    }
    EXPECT_GT(plus[0], 0.0);
    EXPECT_GT(plus[1], 0.0);
}

/** Ackerman isolation independently of the pending static-geometry acceptance. */
TEST(L5RackTravel, T8_AckermanDoesNotChangeRackOutput) {
    for (double rack : {-kRackM, kRackM}) {
        std::array<std::vector<double>, 2> values;
        for (int j = 0; j < 2; ++j) {
            auto vp = sedan(); vp.steer_input = "rack_travel";
            vp.ackerman_percent = j ? 100.0 : 0.0;
            auto session = make_l5(vp, true);
            inject_rack(*session, 0.0);
            tick_n(*session, CmdL4{}, 200);
            inject_rack(*session, rack);
            tick_n(*session, CmdL4{}, 100);
            values[j] = dump(*session);
            for (double x : free_3d_wheel_toe(session->dynamics())) values[j].push_back(x);
        }
        ASSERT_EQ(values[0].size(), values[1].size());
        EXPECT_EQ(0, std::memcmp(values[0].data(), values[1].data(), values[0].size()*sizeof(double)));
    }
}

TEST(L5RackTravel, T9_DisableEnvFailsT5ThenRestores) {
    VehicleParams vp = sedan();
    vp.steer_input = "rack_travel";

    auto s0 = make_l5(vp, true);
    inject_rack(*s0, 0.0);
    tick_n(*s0, CmdL4{}, 300);
    const auto base = toe_deg(s0->dynamics());

    set_disable_env(true);
    {
        auto s = make_l5(vp, true);
        tick_n(*s, CmdL4{}, 200);
        CmdL4 u;
        inject_rack(*s, +kRackM);
        tick_n(*s, u, 100);
        auto t = toe_deg(s->dynamics());
        const double dfl = t[0] - base[0];
        const double band = std::abs(kT5[0][0]) * kTol;
        EXPECT_GT(std::abs(dfl - kT5[0][0]), band) << "T9 disabled path must miss the table";
        std::printf("T9 disabled dFL=%.6f (table %.6f)\n", dfl, kT5[0][0]);
    }
    set_disable_env(false);
    {
        auto s = make_l5(vp, true);
        tick_n(*s, CmdL4{}, 200);
        CmdL4 u;
        inject_rack(*s, +kRackM);
        tick_n(*s, u, 100);
        auto t = toe_deg(s->dynamics());
        const double dfl = t[0] - base[0];
        const double band = std::max(1e-6, std::abs(kT5[0][0]) * kTol);
        EXPECT_NEAR(dfl, kT5[0][0], band);
    }
}
