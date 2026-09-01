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

namespace {

// Richardson-extrapolated probe channels.  dt = 1e-8 (P3-6): at 1e-10 the two
// runs sit 1 ULP (3.6e-12) apart, so the extrapolation measures rounding
// instead of accuracy.
constexpr double kProbeDt = 1e-8;
constexpr double kEomClosureTolerance = 1e-6;   // [m/s^2]
constexpr double kLoadToleranceN = 1e-6;        // [N]

struct RoadProbe {
    std::array<double, vdsim::NUM_WHEELS> fz {{0.0, 0.0, 0.0, 0.0}};
    double sum_fz {0.0};
    double tire_fy {0.0};
    double dvy {0.0};
    double ay {0.0};
    double gy_state {0.0};
    double gy_ay {0.0};
    double gy_state_coarse {0.0};
    double gy_state_fine {0.0};
    double gy_ay_coarse {0.0};
    double gy_ay_fine {0.0};
    double vx {0.0};
    double r {0.0};
};

RoadProbe probe_contacts_dt(bool l4,
                            const vdsim::ContactArray& contacts,
                            const vdsim::State& initial,
                            double dt) {
    const double mass = vdsim::VehicleParams{}.mass;
    auto coarse = make_level(l4, initial);
    auto fine = make_level(l4, initial);
    coarse->step(vdsim::CmdL4{}, contacts, dt);
    fine->step(vdsim::CmdL4{}, contacts, 0.5 * dt);
    const auto extrapolate = [](double c, double f) { return 2.0 * f - c; };
    RoadProbe probe;
    probe.vx = initial.velocity.x();
    probe.r = initial.angular_velocity.z();
    const double vy0 = initial.velocity.y();
    double coarse_fy = 0.0;
    double fine_fy = 0.0;
    for (int wheel = 0; wheel < vdsim::NUM_WHEELS; ++wheel) {
        probe.fz[wheel] = extrapolate(coarse->tire_Fz()[wheel],
                                      fine->tire_Fz()[wheel]);
        probe.sum_fz += probe.fz[wheel];
        coarse_fy += coarse->tire_forces_body()[wheel].y();
        fine_fy += fine->tire_forces_body()[wheel].y();
    }
    probe.tire_fy = extrapolate(coarse_fy, fine_fy);
    const double coarse_dvy = (coarse->state().velocity.y() - vy0) / dt;
    const double fine_dvy = (fine->state().velocity.y() - vy0) / (0.5 * dt);
    probe.dvy = extrapolate(coarse_dvy, fine_dvy);
    probe.ay = extrapolate(coarse->ay_body_est(), fine->ay_body_est());
    probe.gy_state_coarse = coarse_dvy + probe.vx * probe.r - coarse_fy / mass;
    probe.gy_state_fine = fine_dvy + probe.vx * probe.r - fine_fy / mass;
    probe.gy_ay_coarse = coarse->ay_body_est() - coarse_fy / mass;
    probe.gy_ay_fine = fine->ay_body_est() - fine_fy / mass;
    probe.gy_state = extrapolate(probe.gy_state_coarse, probe.gy_state_fine);
    probe.gy_ay = extrapolate(probe.gy_ay_coarse, probe.gy_ay_fine);
    return probe;
}

RoadProbe probe_contacts(bool l4,
                         const vdsim::ContactArray& contacts,
                         const vdsim::State& initial) {
    return probe_contacts_dt(l4, contacts, initial, kProbeDt);
}

vdsim::Vec3 averaged_normal(const vdsim::ContactArray& contacts) {
    vdsim::Vec3 n_road = vdsim::Vec3::Zero();
    for (const auto& contact : contacts) {
        if (!contact.is_valid) continue;
        n_road += contact.normal / contact.normal.norm();
    }
    return n_road.normalized();
}

double road_gravity_y(const vdsim::ContactArray& contacts) {
    const vdsim::Vec3 n_road = averaged_normal(contacts);
    vdsim::Vec3 x_road = vdsim::Vec3::UnitX();
    x_road -= x_road.dot(n_road) * n_road;
    x_road.normalize();
    return -kGravity * n_road.cross(x_road).normalized().z();
}

}  // namespace

TEST(D2RoadLoads, BankedTurnMatchesIndependentTextbookFrictionLimit) {
    constexpr double bank = 10.0 * kPi / 180.0;
    constexpr double mu = 0.80;
    constexpr double radius_m = 60.0;
    const double limit_accel = kGravity * (std::sin(bank) + mu * std::cos(bank))
                             / (std::cos(bank) - mu * std::sin(bank));
    const double speed = std::sqrt(radius_m * limit_accel);
    const auto contacts = contacts_with_normal(
        vdsim::Vec3(0.0, std::sin(bank), std::cos(bank)), mu);
    vdsim::State initial;
    initial.velocity.x() = speed;
    initial.angular_velocity.z() = speed / radius_m;
    const double omega = speed / vdsim::VehicleParams{}.wheel_radius_nominal;
    initial.wheel_spin = {{omega, omega, omega, omega}};

    // Independent analytical force balance, without deriving any expected
    // quantity from the simulated normal load:
    //   N cos(b) - T sin(b) = m g
    //   N sin(b) + T cos(b) = m v^2/R
    // At the friction limit T = mu N, yielding the speed above and
    //   N = m(g cos(b) + (v^2/R) sin(b)).
    const double mass = vdsim::VehicleParams{}.mass;
    const double expected_normal = mass
        * (kGravity * std::cos(bank) + limit_accel * std::sin(bank));
    const double required_tangent = mass
        * (limit_accel * std::cos(bank) - kGravity * std::sin(bank));

    for (const bool l4 : {false, true}) {
        const RoadProbe p = probe_contacts(l4, contacts, initial);
        const double normal_rel_error = std::abs(
            (p.sum_fz - expected_normal) / expected_normal);
        const double friction_residual = required_tangent - mu * p.sum_fz;
        EXPECT_NEAR(p.sum_fz, expected_normal, kLoadToleranceN);
        EXPECT_NEAR(friction_residual, 0.0, kLoadToleranceN);
        std::cout << std::setprecision(17)
                  << "[D2:bank-limit] l4=" << l4
                  << " expected_speed_mps=" << speed
                  << " expected_sumFz_N=" << expected_normal
                  << " observed_sumFz_N=" << p.sum_fz
                  << " required_tangent_N=" << required_tangent
                  << " mu_times_observed_sumFz_N=" << mu * p.sum_fz
                  << " friction_residual_N=" << friction_residual
                  << " normal_rel_error=" << normal_rel_error << '\n';
    }
}

TEST(D2RoadLoads, HeterogeneousNormalsDoNotLeakSpuriousPlanarForce) {
    constexpr double bank = 30.0 * kPi / 180.0;
    auto contacts = contacts_with_normal(vdsim::Vec3::UnitZ());
    contacts[vdsim::WHEEL_FL].normal =
        vdsim::Vec3(0.0, std::sin(bank), std::cos(bank));
    const double expected_gy = road_gravity_y(contacts);

    vdsim::State initial;
    for (const bool l4 : {false, true}) {
        const RoadProbe p = probe_contacts(l4, contacts, initial);
        const double state_residual = p.gy_state - expected_gy;
        const double reported_residual = p.gy_ay - expected_gy;
        EXPECT_NEAR(state_residual, 0.0, kEomClosureTolerance);
        EXPECT_NEAR(reported_residual, 0.0, kEomClosureTolerance);
        std::cout << std::setprecision(17)
                  << "[D2:heterogeneous-normal] l4=" << l4
                  << " expected_gravity_y_mps2=" << expected_gy
                  << " observed_state_gravity_y_mps2=" << p.gy_state
                  << " observed_reported_gravity_y_mps2=" << p.gy_ay
                  << " state_residual_mps2=" << state_residual
                  << " reported_residual_mps2=" << reported_residual
                  << " sumFz_N=" << p.sum_fz << '\n';
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
