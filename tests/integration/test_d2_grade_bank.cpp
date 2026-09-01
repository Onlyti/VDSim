#include "vdsim/interfaces.hpp"
#include "vdsim/suspension.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

/** Construct the real L3 or shipped-topology L4 vehicle path without aero. */
std::unique_ptr<vdsim::IVehicleDynamics> make_level(bool l4,
                                                     const vdsim::State& initial) {
    auto dynamics = l4 ? vdsim::create_fourteen_dof_kinematic()
                       : vdsim::create_fourteen_dof();
    vdsim::VehicleParams vehicle;
    vehicle.aero_drag_coeff = 0.0;
    vehicle.aero_lift_front = 0.0;
    vehicle.aero_lift_rear = 0.0;
    vehicle.plant_path = true;
    dynamics->initialize(vehicle, vdsim::TireParams{}, vdsim::SolverParams{});
    dynamics->reset(initial);
    return dynamics;
}

/** Build four valid contacts sharing one exact world-ENU unit normal. */
vdsim::ContactArray contacts_with_normal(const vdsim::Vec3& normal,
                                         double mu = 1.0) {
    vdsim::ContactArray contacts{};
    for (auto& contact : contacts) {
        contact.is_valid = true;
        contact.normal = normal;
        contact.mu_long = mu;
        contact.mu_lat = mu;
    }
    return contacts;
}

/** Record every travel value handed to the existing camber/toe sweep API. */
class TravelRecorder final : public vdsim::ISuspensionKinematics {
public:
    explicit TravelRecorder(std::shared_ptr<std::vector<double>> samples)
        : samples_(std::move(samples)) {}

    /** Record travel and return a deterministic linear camber/toe sweep. */
    Output compute(double wheel_travel, double) const noexcept override {
        samples_->push_back(wheel_travel);
        Output out;
        out.camber = 2.0 * wheel_travel;
        out.toe = -3.0 * wheel_travel;
        return out;
    }

private:
    std::shared_ptr<std::vector<double>> samples_;
};

}  // namespace

TEST(D2RoadLoads, StaticGradeUsesExactTangentialGravityAndCosineNormalLoad) {
    constexpr double theta = 12.0 * kPi / 180.0;
    // The public L3/L4 path rejects dt=0.  Evaluate at h and h/2 and use first-
    // order Richardson extrapolation, 2*y(h/2)-y(h), to remove the measured
    // O(h) tire-spin perturbation from the analytical zero-time force balance.
    constexpr double dt = 1e-8;
    const vdsim::Vec3 normal(-std::sin(theta), 0.0, std::cos(theta));
    const auto contacts = contacts_with_normal(normal);
    const vdsim::VehicleParams vehicle;

    for (const bool l4 : {false, true}) {
        vdsim::State initial;
        auto coarse = make_level(l4, initial);
        auto fine = make_level(l4, initial);
        coarse->step(vdsim::CmdL4{}, contacts, dt);
        fine->step(vdsim::CmdL4{}, contacts, 0.5 * dt);

        double coarse_fz = 0.0, fine_fz = 0.0;
        for (double fz : coarse->tire_Fz()) coarse_fz += fz;
        for (double fz : fine->tire_Fz()) fine_fz += fz;
        const double coarse_ax = coarse->state().velocity.x() / dt;
        const double fine_ax = fine->state().velocity.x() / (0.5 * dt);
        const double observed_ax = 2.0 * fine_ax - coarse_ax;
        const double sum_fz = 2.0 * fine_fz - coarse_fz;
        const double expected_ax = -kGravity * std::sin(theta);
        const double expected_fz = vehicle.mass * kGravity * std::cos(theta);
        const double ax_rel_error = std::abs((observed_ax - expected_ax) / expected_ax);
        const double fz_rel_error = std::abs((sum_fz - expected_fz) / expected_fz);

        EXPECT_NEAR(observed_ax, expected_ax, 2e-9);
        EXPECT_NEAR(sum_fz, expected_fz, 2e-9);
        std::cout << std::setprecision(17)
                  << "[D2:grade] l4=" << l4
                  << " expected_ax_mps2=" << expected_ax
                  << " observed_ax_mps2=" << observed_ax
                  << " coarse_ax_mps2=" << coarse_ax
                  << " fine_ax_mps2=" << fine_ax
                  << " ax_rel_error=" << ax_rel_error
                  << " expected_sumFz_N=" << expected_fz
                  << " observed_sumFz_N=" << sum_fz
                  << " Fz_rel_error=" << fz_rel_error << '\n';
    }
}

TEST(D2RoadLoads, BankedTurnNormalLoadClosesTextbookFrictionLimit) {
    constexpr double bank = 10.0 * kPi / 180.0;
    constexpr double mu = 0.80;
    constexpr double radius_m = 60.0;
    constexpr double dt = 1e-10;
    const double sin_b = std::sin(bank);
    const double cos_b = std::cos(bank);

    // Textbook banked-curve limit, derived from the actual normal and tangential
    // force balances (not an equivalent-ay correction):
    //   N*cos(bank) - mu*N*sin(bank) = m*g
    //   N*sin(bank) + mu*N*cos(bank) = m*v^2/R
    //   v_max^2 = R*g*(sin(bank)+mu*cos(bank))/(cos(bank)-mu*sin(bank)).
    const double expected_a = kGravity * (sin_b + mu * cos_b)
                            / (cos_b - mu * sin_b);
    const double expected_speed = std::sqrt(radius_m * expected_a);
    const double expected_normal = vdsim::VehicleParams{}.mass
                                 * (kGravity * cos_b + expected_a * sin_b);
    const vdsim::Vec3 normal(0.0, sin_b, cos_b);
    const auto contacts = contacts_with_normal(normal, mu);

    for (const bool l4 : {false, true}) {
        vdsim::State initial;
        initial.velocity.x() = expected_speed;
        initial.angular_velocity.z() = expected_speed / radius_m;
        const double omega = expected_speed
                           / vdsim::VehicleParams{}.wheel_radius_nominal;
        initial.wheel_spin = {{omega, omega, omega, omega}};
        auto coarse = make_level(l4, initial);
        auto fine = make_level(l4, initial);
        coarse->step(vdsim::CmdL4{}, contacts, dt);
        fine->step(vdsim::CmdL4{}, contacts, 0.5 * dt);

        double coarse_normal = 0.0, fine_normal = 0.0;
        for (double fz : coarse->tire_Fz()) coarse_normal += fz;
        for (double fz : fine->tire_Fz()) fine_normal += fz;
        const double observed_normal = 2.0 * fine_normal - coarse_normal;
        const double observed_a = observed_normal
            * (sin_b + mu * cos_b) / vdsim::VehicleParams{}.mass;
        const double observed_speed = std::sqrt(radius_m * observed_a);
        const double vertical_residual = observed_normal * (cos_b - mu * sin_b)
                                       - vdsim::VehicleParams{}.mass * kGravity;
        const double relative_error = std::abs(
            (observed_speed - expected_speed) / expected_speed);

        EXPECT_NEAR(observed_normal, expected_normal, 2e-8);
        EXPECT_NEAR(vertical_residual, 0.0, 2e-8);
        EXPECT_NEAR(observed_speed, expected_speed, 2e-11);
        std::cout << std::setprecision(17)
                  << "[D2:bank-limit] l4=" << l4
                  << " expected_speed_mps=" << expected_speed
                  << " observed_speed_mps=" << observed_speed
                  << " expected_sumFz_N=" << expected_normal
                  << " observed_sumFz_N=" << observed_normal
                  << " coarse_sumFz_N=" << coarse_normal
                  << " fine_sumFz_N=" << fine_normal
                  << " vertical_residual_N=" << vertical_residual
                  << " relative_error=" << relative_error << '\n';
    }
}

TEST(D2RoadLoads, L4RoadHeightDrivesExistingTravelCamberToeSweep) {
    vdsim::State initial;
    auto dynamics = make_level(true, initial);
    auto samples = std::make_shared<std::vector<double>>();
    ASSERT_TRUE(vdsim::attach_front_kinematics(
        *dynamics, std::make_unique<TravelRecorder>(samples)));

    auto contacts = contacts_with_normal(vdsim::Vec3::UnitZ());
    contacts[vdsim::WHEEL_FL].road_dz = 0.010;
    contacts[vdsim::WHEEL_FR].road_dz = -0.010;
    for (int step = 0; step < 20; ++step) {
        dynamics->step(vdsim::CmdL4{}, contacts, 0.001);
    }

    ASSERT_GE(samples->size(), 40U);
    // Calls are FL then FR on every step.  The first call pair is the initial
    // zero-travel state; later samples must carry the opposite road-height input
    // through z_u -> wheel_travel -> this existing sweep API.
    EXPECT_DOUBLE_EQ((*samples)[0], 0.0);
    EXPECT_DOUBLE_EQ((*samples)[1], 0.0);
    const double fl_travel = (*samples)[samples->size() - 2];
    const double fr_travel = (*samples)[samples->size() - 1];
    EXPECT_GT(fl_travel, 1e-5);
    EXPECT_LT(fr_travel, -1e-5);
    EXPECT_NEAR(fl_travel, -fr_travel, 2e-9);
    std::cout << std::setprecision(17)
              << "[D2:L4-height-sweep] fl_travel_m=" << fl_travel
              << " fr_travel_m=" << fr_travel
              << " sample_count=" << samples->size() << '\n';
}
