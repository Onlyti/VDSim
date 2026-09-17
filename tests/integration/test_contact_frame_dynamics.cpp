#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/** Extract ISO body roll/pitch from a body-to-world quaternion. */
std::array<double, 2> roll_pitch(const vdsim::Quat& q) {
    const double roll = std::atan2(
        2.0 * (q.w() * q.x() + q.y() * q.z()),
        1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    const double sin_pitch = std::clamp(
        2.0 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0);
    return {{roll, std::asin(sin_pitch)}};
}

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

    // Rebuilding a level from a State snapshot is an exact clone only when the
    // whole history lives in that State. At L4 it does not: the KC bushing
    // compliance keeps its history in the axle hard-joint DAE. That residual
    // belongs to the reconstruction, not to the airborne step, so it is
    // measured on a control run that never loses contact and used as the
    // bound. Picking a numeric tolerance from the observed failure instead
    // would only record that the difference exists.
    struct Residual {
        std::array<double, vdsim::NUM_WHEELS> force{};
        std::array<double, vdsim::NUM_WHEELS> kappa{};
        std::array<double, vdsim::NUM_WHEELS> alpha{};
    };
    const auto measure = [&](bool l4, bool go_airborne) {
        Residual residual{};
        auto warm = make_level(l4, true);
        EXPECT_NE(warm, nullptr);
        if (warm == nullptr) return residual;
        for (int i = 0; i < 100; ++i) {
            warm->step(vdsim::CmdL4{}, valid, 0.001);
        }
        if (go_airborne) warm->step(vdsim::CmdL4{}, invalid, 0.001);
        const vdsim::State recontact_state = warm->state();
        auto fresh = make_level(l4, true, &recontact_state);
        EXPECT_NE(fresh, nullptr);
        if (fresh == nullptr) return residual;

        warm->step(vdsim::CmdL4{}, valid, 1e-10);
        fresh->step(vdsim::CmdL4{}, valid, 1e-10);
        for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
            residual.force[wheel] = (warm->tire_forces_body()[wheel]
                                     - fresh->tire_forces_body()[wheel]).norm();
            residual.kappa[wheel] = std::abs(warm->wheel_slip_ratio()[wheel]
                                             - fresh->wheel_slip_ratio()[wheel]);
            residual.alpha[wheel] = std::abs(warm->wheel_slip_angle()[wheel]
                                             - fresh->wheel_slip_angle()[wheel]);
        }
        return residual;
    };

    for (const bool l4 : {false, true}) {
        const Residual airborne = measure(l4, true);
        const Residual control = measure(l4, false);
        for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
            // Losing contact must leave nothing behind: after the airborne
            // step the warm level and a level rebuilt from its state agree at
            // least as closely as they do when contact was never lost.
            EXPECT_LE(airborne.force[wheel],
                      std::max(control.force[wheel], 1e-8))
                << "l4=" << l4 << " wheel=" << wheel;
            EXPECT_LE(airborne.kappa[wheel],
                      std::max(control.kappa[wheel], 2e-10))
                << "l4=" << l4 << " wheel=" << wheel;
            EXPECT_LE(airborne.alpha[wheel],
                      std::max(control.alpha[wheel], 3e-9))
                << "l4=" << l4 << " wheel=" << wheel;
        }
    }
}

TEST(D1ContactFrame, BankVerticalDeltaEntersUnsprungForceBalanceOnce) {
    constexpr double bank = 10.0 * kDegToRad;
    constexpr double dt = 5e-8;
    // D2 resolves gravity along the exact road-y basis (g*sin(bank)) instead of
    // its former horizontal projection (g*sin(bank)*cos(bank)); this is the new
    // physical non-flat baseline while the D1 wrench closure itself is unchanged.
    constexpr double expected_front_left_delta_N = -435.6222462;
    // Central model state is observed after a finite RK step.  A dt sweep at
    // 1e-7/5e-8 bounded the reconstruction residual by 1.56e-3 N; 2e-3 N is
    // the measured finite-difference envelope, not a model-error allowance.
    constexpr double force_balance_tolerance_N = 2e-3;

    auto flat = make_level(true);
    auto plus = make_level(true);
    const auto force_probe = evaluate_bank(true, bank);
    ASSERT_NE(flat, nullptr);
    ASSERT_NE(plus, nullptr);
    ASSERT_NE(force_probe, nullptr);

    vdsim::ContactArray flat_contacts{};
    vdsim::ContactArray bank_contacts{};
    for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
        flat_contacts[wheel].is_valid = true;
        flat_contacts[wheel].normal = vdsim::Vec3::UnitZ();
        flat_contacts[wheel].mu_long = 1.0;
        flat_contacts[wheel].mu_lat = 1.0;
        bank_contacts[wheel] = flat_contacts[wheel];
        bank_contacts[wheel].normal =
            vdsim::Vec3(0.0, -std::sin(bank), std::cos(bank));
    }

    flat->step(vdsim::CmdL4{}, flat_contacts, dt);
    plus->step(vdsim::CmdL4{}, bank_contacts, dt);

    const vdsim::VehicleParams vehicle;
    const std::array<double, vdsim::NUM_WHEELS> rx {{
        vehicle.cg_to_front, vehicle.cg_to_front,
       -vehicle.cg_to_rear, -vehicle.cg_to_rear}};
    const std::array<double, vdsim::NUM_WHEELS> ry {{
        0.5 * vehicle.track_front, -0.5 * vehicle.track_front,
        0.5 * vehicle.track_rear, -0.5 * vehicle.track_rear}};
    const vdsim::Vec3 angular_rate_delta =
        plus->state().angular_velocity - flat->state().angular_velocity;
    for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
        // Public tire_forces_body is tangential-only.  Its z component is the
        // contact-plane Fy projection, so the omitted normal contribution is
        // Fz*cos(bank); subtract the legacy Fz*e_z exactly once.
        const auto tangential = force_probe->tire_forces_body()[wheel];
        const double expected_delta = tangential.z()
            + force_probe->tire_Fz()[wheel] * (std::cos(bank) - 1.0);
        // susp_velocity = z_u_dot - z_s_dot - ry*phi_dot + rx*theta_dot.
        // At the first infinitesimal step, z_s_dot is identical because the
        // contact increment acts on the unsprung mass.  Remove the independently
        // observed roll/pitch-rate terms to recover the unsprung acceleration.
        const double observed_unsprung_velocity_delta =
            plus->state().susp_velocity[wheel]
            - flat->state().susp_velocity[wheel]
            + ry[wheel] * angular_rate_delta.x()
            - rx[wheel] * angular_rate_delta.y();
        const double observed_delta = vehicle.unsprung_mass[wheel]
            * observed_unsprung_velocity_delta / dt;

        EXPECT_NEAR(tangential.dot(bank_contacts[wheel].normal), 0.0, 1e-9);
        EXPECT_LT(expected_delta, 0.0);
        EXPECT_LT(observed_delta, 0.0);
        EXPECT_NEAR(observed_delta, expected_delta, force_balance_tolerance_N)
            << "wheel=" << wheel;
        if (wheel == vdsim::WHEEL_FL) {
            EXPECT_NEAR(expected_delta, expected_front_left_delta_N, 1e-6);
            std::cout << std::setprecision(17)
                      << "[D1:vertical] expected_delta_N=" << expected_delta
                      << " observed_balance_N=" << observed_delta
                      << " virtual_power_at_1mps_W=" << expected_delta << '\n';
        }
    }
}

TEST(D1ContactFrame, WarmedRollPitchUsesWorldZLegacyLoadAxis) {
    constexpr double bank = 10.0 * kDegToRad;
    constexpr double warm_dt = 0.001;
    constexpr double probe_dt = 5e-8;
    auto flat = make_level(true);
    auto plus = make_level(true);
    ASSERT_NE(flat, nullptr);
    ASSERT_NE(plus, nullptr);

    vdsim::ContactArray warm_contacts{};
    for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
        auto& contact = warm_contacts[wheel];
        contact.is_valid = true;
        contact.normal = vdsim::Vec3::UnitZ();
        contact.mu_long = 1.0;
        contact.mu_lat = 1.0;
        const bool front = wheel == vdsim::WHEEL_FL || wheel == vdsim::WHEEL_FR;
        contact.road_dz = front ? 0.0022 : -0.0022;
    }
    vdsim::CmdL4 warm_command;
    warm_command.steer_angle_wheel = 0.06;
    for (int step = 0; step < 4000; ++step) {
        flat->step(warm_command, warm_contacts, warm_dt);
        plus->step(warm_command, warm_contacts, warm_dt);
    }

    const auto warm_rp = roll_pitch(plus->state().orientation);
    const vdsim::Quat warm_body_to_world = plus->state().orientation;
    const vdsim::Vec3 warm_rate_delta =
        plus->state().angular_velocity - flat->state().angular_velocity;
    EXPECT_NEAR(warm_rp[0], 0.05, 0.01);
    EXPECT_NEAR(warm_rp[1], -0.00116, 0.001);
    EXPECT_NEAR(warm_rate_delta.norm(), 0.0, 1e-14);

    vdsim::ContactArray bank_contacts = warm_contacts;
    const vdsim::Vec3 bank_normal(0.0, -std::sin(bank), std::cos(bank));
    for (auto& contact : bank_contacts) contact.normal = bank_normal;
    flat->step(warm_command, warm_contacts, probe_dt);
    plus->step(warm_command, bank_contacts, probe_dt);

    const vdsim::VehicleParams vehicle;
    const std::array<double, vdsim::NUM_WHEELS> rx {{
        vehicle.cg_to_front, vehicle.cg_to_front,
       -vehicle.cg_to_rear, -vehicle.cg_to_rear}};
    const std::array<double, vdsim::NUM_WHEELS> ry {{
        0.5 * vehicle.track_front, -0.5 * vehicle.track_front,
        0.5 * vehicle.track_rear, -0.5 * vehicle.track_rear}};
    const vdsim::Vec3 rate_delta =
        plus->state().angular_velocity - flat->state().angular_velocity;
    const double Rzz = (warm_body_to_world * vdsim::Vec3::UnitZ()).z();

    for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
        const vdsim::Vec3 tangential_world = warm_body_to_world
            * plus->tire_forces_body()[wheel];
        const double expected_world_delta = tangential_world.z()
            + plus->tire_Fz()[wheel] * (bank_normal.z() - 1.0);
        const double observed_unsprung_velocity_delta =
            plus->state().susp_velocity[wheel]
            - flat->state().susp_velocity[wheel]
            + ry[wheel] * rate_delta.x() - rx[wheel] * rate_delta.y();
        const double observed_world_delta = vehicle.unsprung_mass[wheel]
            * observed_unsprung_velocity_delta / probe_dt;
        EXPECT_NEAR(observed_world_delta, expected_world_delta, 3e-3)
            << "wheel=" << wheel;

        const double old_body_axis_residual =
            plus->tire_Fz()[wheel] * (1.0 - Rzz);
        if (wheel == vdsim::WHEEL_FL) {
            // For a ZYX body attitude, e_z(body) dot e_z(world) is
            // cos(roll)*cos(pitch).  The legacy body-z load axis therefore
            // drops this exact fraction of Fz from the world-z balance.  This
            // dimensionless geometry check is independent of tire load and
            // replaces the former warmed-state force snapshot.
            const double expected_axis_loss_fraction =
                1.0 - std::cos(warm_rp[0]) * std::cos(warm_rp[1]);
            const double observed_axis_loss_fraction =
                old_body_axis_residual / plus->tire_Fz()[wheel];
            EXPECT_GT(expected_axis_loss_fraction, 0.0);
            EXPECT_GT(old_body_axis_residual, 0.0);
            EXPECT_NEAR(1.0 - Rzz, expected_axis_loss_fraction, 1e-14);
            EXPECT_NEAR(observed_axis_loss_fraction,
                        expected_axis_loss_fraction, 1e-14);
            std::cout << std::setprecision(17)
                      << "[D1:warm-axis] roll_rad=" << warm_rp[0]
                      << " pitch_rad=" << warm_rp[1]
                      << " expected_world_delta_N=" << expected_world_delta
                      << " observed_world_delta_N=" << observed_world_delta
                      << " old_body_axis_residual_N=" << old_body_axis_residual
                      << '\n';
        }
    }
}
