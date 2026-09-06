// D0 flat-road equivalence gate for the future C2 contact-frame work.
//
// This test intentionally changes no core behavior. It freezes the current L3
// response for normal=(0,0,1), road_dz=0 and compares future implementations
// against raw state, force and output samples.
//
// Fixture provenance: the L3 fixture has never moved. The L4 fixture was
// re-captured on integration/p0-merge because main 9a40c1b (KC bushing
// compliance under lateral tire load) changes the flat-road L4 response for a
// reason that has nothing to do with the contact-frame work. That attribution
// was measured, not assumed: with compliance_targets_rad() suppressed, the
// merged tree reproduces the pre-merge L4 fixture bit-for-bit, so the D0
// equivalence claim still holds on this tree and only its reference moved.
// Re-capture with VDSIM_D0_CAPTURE_L3 / VDSIM_D0_CAPTURE_L4 and say why.

#include "vdsim/interfaces.hpp"
#include "vdsim/multibody.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 0.005;
constexpr int kFinalStep = 400;
constexpr double kStateOutputTolerance = 1e-12;
constexpr double kForceToleranceN = 1e-9;
constexpr std::array<int, 7> kCheckpoints{{0, 1, 20, 100, 200, 300, 400}};

using Row = std::vector<double>;
using Table = std::vector<Row>;

/** Return the stable CSV schema grouped as state, force, and output signals. */
std::vector<std::string> headers() {
    std::vector<std::string> h{
        "step", "t_s",
        "state_pos_x_m", "state_pos_y_m", "state_pos_z_m",
        "state_q_w", "state_q_x", "state_q_y", "state_q_z",
        "state_vx_mps", "state_vy_mps", "state_vz_mps",
        "state_roll_rate_radps", "state_pitch_rate_radps", "state_yaw_rate_radps",
    };
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        h.push_back(std::string("state_wheel_spin_") + wheel + "_radps");
    }
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        h.push_back(std::string("state_susp_compression_") + wheel + "_m");
    }
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        h.push_back(std::string("state_susp_velocity_") + wheel + "_mps");
    }
    h.push_back("state_rack_travel_m");
    h.push_back("state_rack_velocity_mps");
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        for (const char* axis : {"x", "y", "z"}) {
            h.push_back(std::string("force_body_") + wheel + "_" + axis + "_N");
        }
    }
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        h.push_back(std::string("force_Fz_") + wheel + "_N");
    }
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        h.push_back(std::string("state_slip_ratio_") + wheel);
    }
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        h.push_back(std::string("state_slip_angle_") + wheel + "_rad");
    }
    for (const char* wheel : {"FL", "FR", "RL", "RR"}) {
        h.push_back(std::string("state_mu_") + wheel);
    }
    h.insert(h.end(), {"output_roll_rad", "output_pitch_rad",
                       "output_ax_mps2", "output_ay_mps2",
                       "output_rack_torque_Nm"});
    return h;
}

/** Append an Eigen 3-vector to a raw row without coordinate conversion. */
void append_vec(Row& row, const vdsim::Vec3& value) {
    row.push_back(value.x());
    row.push_back(value.y());
    row.push_back(value.z());
}

/** Append a fixed-size wheel array to a raw row in FL,FR,RL,RR order. */
template <typename T>
void append_wheels(Row& row, const std::array<T, vdsim::NUM_WHEELS>& values) {
    for (const auto& value : values) row.push_back(static_cast<double>(value));
}

/** Capture one checkpoint's state, force diagnostics, and public outputs. */
Row capture(const vdsim::IVehicleDynamics& dyn, int step) {
    const auto& state = dyn.state();
    Row row;
    row.reserve(headers().size());
    row.push_back(static_cast<double>(step));
    row.push_back(step * kDt);
    append_vec(row, state.position);
    row.insert(row.end(), {state.orientation.w(), state.orientation.x(),
                           state.orientation.y(), state.orientation.z()});
    append_vec(row, state.velocity);
    append_vec(row, state.angular_velocity);
    append_wheels(row, state.wheel_spin);
    append_wheels(row, state.susp_compression);
    append_wheels(row, state.susp_velocity);
    row.push_back(state.rack_travel);
    row.push_back(state.rack_velocity);
    for (const auto& force : dyn.tire_forces_body()) append_vec(row, force);
    append_wheels(row, dyn.tire_Fz());
    append_wheels(row, dyn.wheel_slip_ratio());
    append_wheels(row, dyn.wheel_slip_angle());
    append_wheels(row, dyn.wheel_mu());
    row.insert(row.end(), {dyn.roll_angle_qs(), dyn.pitch_angle_qs(),
                           dyn.ax_body_est(), dyn.ay_body_est(),
                           dyn.steering_rack_torque()});
    return row;
}

/** Return the deterministic three-phase command at one integration step. */
vdsim::ControlInput command_at(int step) {
    const double t = step * kDt;
    vdsim::CmdL4 command;
    if (t < 0.5) {
        command.throttle = 0.20;
    } else if (t < 1.5) {
        command.throttle = 0.15;
        command.steer_angle_wheel = 0.06;
    } else {
        command.brake = 0.25;
        command.steer_angle_wheel = 0.02;
    }
    return command;
}

/** Execute the representative L3 or production-configured L4 flat-road path. */
Table run_scenario(bool use_l4) {
    vdsim::VehicleParams vehicle;
    vehicle.aero_drag_coeff = 0.0;
    vdsim::TireParams tire;
    tire.lugre.enabled = false;
    vdsim::SolverParams solver;
    auto dynamics = use_l4
        ? vdsim::create_fourteen_dof_kinematic()
        : vdsim::create_fourteen_dof();
    dynamics->initialize(vehicle, tire, solver);
    if (use_l4) {
        const std::string topology_path = std::string(VDSIM_SOURCE_DIR)
            + "/configs/parts/susp_kinematics/kin/mp_front_sedan.yaml";
        const auto topology =
            vdsim::mb::SuspensionTopology::from_yaml(topology_path);
        if (!vdsim::mb::attach_topology_front(*dynamics, topology)) {
            throw std::runtime_error("cannot attach production L4 front topology");
        }
    }

    vdsim::State initial;
    initial.velocity.x() = 15.0;
    const double omega = initial.velocity.x() / vehicle.wheel_radius_nominal;
    initial.wheel_spin = {{omega, omega, omega, omega}};
    dynamics->reset(initial);

    vdsim::ContactArray contacts{};
    for (auto& contact : contacts) {
        contact.is_valid = true;
        contact.normal = vdsim::Vec3::UnitZ();
        contact.road_dz = 0.0;
        contact.mu_long = 1.0;
        contact.mu_lat = 1.0;
    }

    Table result;
    for (int step = 0; step <= kFinalStep; ++step) {
        if (std::find(kCheckpoints.begin(), kCheckpoints.end(), step)
                != kCheckpoints.end()) {
            result.push_back(capture(*dynamics, step));
        }
        if (step < kFinalStep) dynamics->step(command_at(step), contacts, kDt);
    }
    return result;
}

/** Write the 17-digit round-trip baseline used by the automated D0 gate. */
void write_csv(const std::string& path, const Table& table) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot open D0 capture path: " + path);
    const auto names = headers();
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i) out << ',';
        out << names[i];
    }
    out << '\n' << std::setprecision(17);
    for (const auto& row : table) {
        for (std::size_t i = 0; i < row.size(); ++i) {
            if (i) out << ',';
            out << row[i];
        }
        out << '\n';
    }
}

/** Read a baseline and require its header to match the compiled signal schema. */
Table read_csv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open D0 baseline: " + path);
    std::string line;
    std::getline(in, line);
    std::ostringstream expected_header;
    const auto names = headers();
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i) expected_header << ',';
        expected_header << names[i];
    }
    if (line != expected_header.str()) {
        throw std::runtime_error("D0 baseline header does not match signal schema");
    }
    Table table;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        Row row;
        std::stringstream stream(line);
        std::string token;
        while (std::getline(stream, token, ',')) row.push_back(std::stod(token));
        if (row.size() != names.size()) {
            throw std::runtime_error("D0 baseline row width mismatch");
        }
        table.push_back(std::move(row));
    }
    return table;
}

/** Return the exact IEEE-754 representation for repeatability comparison. */
std::uint64_t bits(double value) {
    std::uint64_t result;
    static_assert(sizeof(result) == sizeof(value), "unexpected double width");
    std::memcpy(&result, &value, sizeof(value));
    return result;
}

/** Identify force channels that use the force-specific absolute tolerance. */
bool is_force_channel(const std::string& name) {
    return name.rfind("force_", 0) == 0;
}

/** Compare one level's repeated run and frozen fixture within the D0 tolerances. */
void verify_level(const char* label,
                  bool use_l4,
                  const char* fixture_name,
                  const char* capture_env,
                  bool allow_legacy_capture_env) {
    const auto current = run_scenario(use_l4);
    const auto repeated = run_scenario(use_l4);
    ASSERT_EQ(current.size(), repeated.size());
    double repeat_max_abs_diff = 0.0;
    bool bit_identical = true;
    for (std::size_t row = 0; row < current.size(); ++row) {
        ASSERT_EQ(current[row].size(), repeated[row].size());
        for (std::size_t col = 0; col < current[row].size(); ++col) {
            ASSERT_TRUE(std::isfinite(current[row][col]));
            repeat_max_abs_diff = std::max(
                repeat_max_abs_diff,
                std::abs(current[row][col] - repeated[row][col]));
            bit_identical = bit_identical
                && bits(current[row][col]) == bits(repeated[row][col]);
        }
    }
    EXPECT_TRUE(bit_identical) << "level=" << label;

    const char* capture_path = std::getenv(capture_env);
    if (capture_path == nullptr && allow_legacy_capture_env) {
        capture_path = std::getenv("VDSIM_D0_CAPTURE");
    }
    if (capture_path != nullptr) {
        ASSERT_NO_THROW(write_csv(capture_path, current));
        std::cout << "[D0:" << label << "] captured=" << capture_path
                  << " rows=" << current.size()
                  << " bit_identical=" << std::boolalpha << bit_identical
                  << " repeat_max_abs_diff=" << repeat_max_abs_diff << '\n';
        return;
    }

    const std::string baseline_path = std::string(VDSIM_SOURCE_DIR)
        + "/tests/fixtures/" + fixture_name;
    const auto baseline = read_csv(baseline_path);
    const auto names = headers();
    ASSERT_EQ(current.size(), baseline.size()) << "level=" << label;
    double state_output_max_abs_diff = 0.0;
    double force_max_abs_diff = 0.0;
    for (std::size_t row = 0; row < current.size(); ++row) {
        ASSERT_EQ(current[row].size(), baseline[row].size());
        for (std::size_t col = 0; col < current[row].size(); ++col) {
            const double difference =
                std::abs(current[row][col] - baseline[row][col]);
            const bool force_channel = is_force_channel(names[col]);
            const double tolerance = force_channel
                ? kForceToleranceN : kStateOutputTolerance;
            if (force_channel) {
                force_max_abs_diff = std::max(force_max_abs_diff, difference);
            } else {
                state_output_max_abs_diff = std::max(
                    state_output_max_abs_diff, difference);
            }
            EXPECT_LE(difference, tolerance)
                << "level=" << label << " row=" << row
                << " channel=" << names[col]
                << " current=" << std::setprecision(17) << current[row][col]
                << " baseline=" << baseline[row][col];
        }
    }
    std::cout << "[D0:" << label << "] bit_identical=" << std::boolalpha
              << bit_identical
              << " repeat_max_abs_diff=" << repeat_max_abs_diff
              << " state_output_max_abs_diff=" << state_output_max_abs_diff
              << " force_max_abs_diff=" << force_max_abs_diff
              << " state_output_tolerance=" << kStateOutputTolerance
              << " force_tolerance_N=" << kForceToleranceN << '\n';
}

}  // namespace

TEST(D0FlatContactBaseline, RepresentativeScenarioMatchesFrozenFixture) {
    verify_level("L3", false, "d0_flat_contact_l3_baseline.csv",
                 "VDSIM_D0_CAPTURE_L3", true);
    verify_level("L4", true, "d0_flat_contact_l4_baseline.csv",
                 "VDSIM_D0_CAPTURE_L4", false);
}
