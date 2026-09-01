#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/** Construct the real L3 or shipped-topology L4 core path. */
std::unique_ptr<vdsim::IVehicleDynamics> make_level(
    bool l4, bool transient = false, const vdsim::State* initial = nullptr) {
    auto dynamics = l4 ? vdsim::create_fourteen_dof_kinematic()
                       : vdsim::create_fourteen_dof();
    vdsim::VehicleParams vehicle;
    vehicle.aero_drag_coeff = 0.0;
    vehicle.plant_path = true;
    vdsim::TireParams tire;
    tire.lugre.enabled = transient;
    tire.belt.enabled = transient;
    tire.belt.sigma_lat = 0.6;
    dynamics->initialize(vehicle, tire, vdsim::SolverParams{});
    if (l4) {
        const std::string path = std::string(VDSIM_SOURCE_DIR)
            + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
        const auto topology = vdsim::mb::SuspensionTopology::from_yaml(path);
        if (!vdsim::mb::attach_topology_front(*dynamics, topology)) return nullptr;
    }
    vdsim::State state = initial ? *initial : vdsim::State{};
    if (!initial) state.velocity = vdsim::Vec3(15.0, 1.0, 0.0);
    const double omega = 15.0 / vehicle.wheel_radius_nominal;
    if (!initial) state.wheel_spin = {{omega, omega, omega, omega}};
    dynamics->reset(state);
    return dynamics;
}

/** Run one near-instantaneous force evaluation at the requested bank angle. */
std::unique_ptr<vdsim::IVehicleDynamics> evaluate_bank(bool l4, double bank_rad) {
    auto dynamics = make_level(l4);
    if (!dynamics) return nullptr;
    vdsim::ContactArray contacts{};
    const vdsim::Vec3 normal(0.0, -std::sin(bank_rad), std::cos(bank_rad));
    for (auto& contact : contacts) {
        contact.is_valid = true;
        contact.normal = normal;
        contact.road_dz = 0.0;
        contact.mu_long = 1.0;
        contact.mu_lat = 1.0;
    }
    dynamics->step(vdsim::CmdL4{}, contacts, 1e-10);
    return dynamics;
}

/** Run an invalid-contact case whose road height must be ignored. */
std::unique_ptr<vdsim::IVehicleDynamics> evaluate_invalid(bool l4, double road_dz) {
    auto dynamics = make_level(l4);
    if (!dynamics) return nullptr;
    vdsim::ContactArray contacts{};
    for (auto& contact : contacts) {
        contact.is_valid = false;
        contact.normal = vdsim::Vec3::Zero();
        contact.road_dz = road_dz;
    }
    for (int i = 0; i < 5; ++i) {
        dynamics->step(vdsim::CmdL4{}, contacts, 0.001);
    }
    return dynamics;
}

}  // namespace

TEST(D1ContactFrame, BankZeroPlusMinusTenHaveTheorySignAndMirrorSymmetry) {
    for (const bool l4 : {false, true}) {
        const auto flat = evaluate_bank(l4, 0.0);
        const auto plus = evaluate_bank(l4, 10.0 * kDegToRad);
        const auto minus = evaluate_bank(l4, -10.0 * kDegToRad);
        ASSERT_NE(flat, nullptr);
        ASSERT_NE(plus, nullptr);
        ASSERT_NE(minus, nullptr);
        for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
            const int mirror_wheel = wheel ^ 1;  // FL<->FR, RL<->RR
            const auto f0 = flat->tire_forces_body()[wheel];
            const auto fp = plus->tire_forces_body()[wheel];
            const auto fm = minus->tire_forces_body()[mirror_wheel];
            EXPECT_LT(f0.y(), 0.0) << "l4=" << l4 << " wheel=" << wheel;
            EXPECT_LT(fp.y(), 0.0) << "l4=" << l4 << " wheel=" << wheel;
            EXPECT_LT(fm.y(), 0.0) << "l4=" << l4 << " wheel=" << wheel;
            EXPECT_NEAR(fp.x(), fm.x(), 2e-6);
            EXPECT_NEAR(fp.y(), fm.y(), 2e-6);
            EXPECT_NEAR(fp.z(), -fm.z(), 2e-6);
            EXPECT_GT(plus->tire_Fz()[wheel], 0.0);
            EXPECT_NEAR(plus->tire_Fz()[wheel],
                        minus->tire_Fz()[mirror_wheel], 1e-9);
            if (wheel == 0) {
                std::cout << std::setprecision(17)
                          << "[D1:bank] l4=" << l4
                          << " plus_force=" << fp.transpose()
                          << " minus_mirror_force=" << fm.transpose()
                          << " plus_Fz=" << plus->tire_Fz()[wheel] << '\n';
            }
        }
    }
}

TEST(D1ContactFrame, InvalidContactIsAirborneAndRoadHeightIndependent) {
    for (const bool l4 : {false, true}) {
        const auto low = evaluate_invalid(l4, 0.0);
        const auto high = evaluate_invalid(l4, 10.0);
        ASSERT_NE(low, nullptr);
        ASSERT_NE(high, nullptr);
        for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
            EXPECT_TRUE(low->tire_forces_body()[wheel].isZero(0.0));
            EXPECT_DOUBLE_EQ(low->tire_Fz()[wheel], 0.0);
            EXPECT_DOUBLE_EQ(low->wheel_slip_ratio()[wheel], 0.0);
            EXPECT_DOUBLE_EQ(low->wheel_slip_angle()[wheel], 0.0);
            EXPECT_DOUBLE_EQ(low->state().susp_compression[wheel],
                             high->state().susp_compression[wheel]);
            EXPECT_DOUBLE_EQ(low->state().susp_velocity[wheel],
                             high->state().susp_velocity[wheel]);
        }
    }
}

TEST(D1ContactFrame, InvalidContactResetsTireTransientBeforeRecontact) {
    vdsim::ContactArray valid{};
    vdsim::ContactArray invalid{};
    for (auto& contact : valid) {
        contact.is_valid = true;
        contact.normal = vdsim::Vec3::UnitZ();
        contact.mu_long = 1.0;
        contact.mu_lat = 1.0;
    }
    for (auto& contact : invalid) {
        contact.is_valid = false;
        contact.normal = vdsim::Vec3::Zero();
    }

    for (const bool l4 : {false, true}) {
        auto warm = make_level(l4, true);
        ASSERT_NE(warm, nullptr);
        for (int i = 0; i < 100; ++i) {
            warm->step(vdsim::CmdL4{}, valid, 0.001);
        }
        warm->step(vdsim::CmdL4{}, invalid, 0.001);
        const vdsim::State recontact_state = warm->state();
        auto fresh = make_level(l4, true, &recontact_state);
        ASSERT_NE(fresh, nullptr);

        warm->step(vdsim::CmdL4{}, valid, 1e-10);
        fresh->step(vdsim::CmdL4{}, valid, 1e-10);
        for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
            EXPECT_NEAR((warm->tire_forces_body()[wheel]
                         - fresh->tire_forces_body()[wheel]).norm(),
                        0.0, 1e-8)
                << "l4=" << l4 << " wheel=" << wheel;
            EXPECT_NEAR(warm->wheel_slip_ratio()[wheel],
                        fresh->wheel_slip_ratio()[wheel], 2e-10);
            EXPECT_NEAR(warm->wheel_slip_angle()[wheel],
                        fresh->wheel_slip_angle()[wheel], 3e-9);
        }
    }
}
