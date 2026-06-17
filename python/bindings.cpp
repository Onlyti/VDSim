// Python bindings for vdsim_core (subset).
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>

#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/control_converter.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/default_subsystems.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/scenario.hpp"
#include "vdsim/state.hpp"
#include "vdsim/multibody.hpp"
#include "vdsim/suspension.hpp"

namespace py = pybind11;

// ---- User-defined subsystem modules: trampolines so Python can subclass the
// C++ interfaces (vdsim/subsystems.hpp) and the model can call back into them.
namespace {
using WheelArray = std::array<double, vdsim::NUM_WHEELS>;
using PairDD     = std::pair<double, double>;

struct PyBrakeSystem : vdsim::IBrakeSystem {
    using vdsim::IBrakeSystem::IBrakeSystem;
    WheelArray wheel_torque(const vdsim::SubsystemContext& ctx) override {
        PYBIND11_OVERRIDE_PURE(WheelArray, vdsim::IBrakeSystem, wheel_torque, ctx);
    }
    void begin_step(const vdsim::SubsystemContext& ctx, double dt) override {
        PYBIND11_OVERRIDE(void, vdsim::IBrakeSystem, begin_step, ctx, dt);
    }
    void reset() override { PYBIND11_OVERRIDE(void, vdsim::IBrakeSystem, reset, ); }
};

struct PySteeringSystem : vdsim::ISteeringSystem {
    using vdsim::ISteeringSystem::ISteeringSystem;
    vdsim::SteeringOutput apply(const vdsim::SubsystemContext& ctx) override {
        PYBIND11_OVERRIDE_PURE(vdsim::SteeringOutput, vdsim::ISteeringSystem, apply, ctx);
    }
    void begin_step(const vdsim::SubsystemContext& ctx, double dt) override {
        PYBIND11_OVERRIDE(void, vdsim::ISteeringSystem, begin_step, ctx, dt);
    }
    void reset() override { PYBIND11_OVERRIDE(void, vdsim::ISteeringSystem, reset, ); }
};

struct PyDrivetrain : vdsim::IDrivetrain {
    using vdsim::IDrivetrain::IDrivetrain;
    vdsim::DrivetrainOutput apply(const vdsim::SubsystemContext& ctx) override {
        PYBIND11_OVERRIDE_PURE(vdsim::DrivetrainOutput, vdsim::IDrivetrain, apply, ctx);
    }
    void begin_step(const vdsim::SubsystemContext& ctx, double dt) override {
        PYBIND11_OVERRIDE(void, vdsim::IDrivetrain, begin_step, ctx, dt);
    }
    void reset() override { PYBIND11_OVERRIDE(void, vdsim::IDrivetrain, reset, ); }
    double engine_rpm()   const override { PYBIND11_OVERRIDE(double, vdsim::IDrivetrain, engine_rpm, ); }
    int    current_gear() const override { PYBIND11_OVERRIDE(int,    vdsim::IDrivetrain, current_gear, ); }
};

struct PySuspension : vdsim::ISuspension {
    using vdsim::ISuspension::ISuspension;
    double force(const vdsim::SubsystemContext& ctx, const vdsim::CornerInput& c) override {
        PYBIND11_OVERRIDE_PURE(double, vdsim::ISuspension, force, ctx, c);
    }
    void reset() override { PYBIND11_OVERRIDE(void, vdsim::ISuspension, reset, ); }
};

struct PyAntiRollBar : vdsim::IAntiRollBar {
    using vdsim::IAntiRollBar::IAntiRollBar;
    PairDD force(const vdsim::SubsystemContext& ctx, const vdsim::AxleDefl& d) override {
        PYBIND11_OVERRIDE_PURE(PairDD, vdsim::IAntiRollBar, force, ctx, d);
    }
    void reset() override { PYBIND11_OVERRIDE(void, vdsim::IAntiRollBar, reset, ); }
};
}  // namespace

PYBIND11_MODULE(vdsim, m) {
    m.doc() = "VDSim core Python bindings";

    // -------- Types --------
    py::enum_<vdsim::VehicleParams::Drive>(m, "Drive")
        .value("FWD", vdsim::VehicleParams::Drive::FWD)
        .value("RWD", vdsim::VehicleParams::Drive::RWD)
        .value("AWD", vdsim::VehicleParams::Drive::AWD)
        .export_values();
    py::enum_<vdsim::VehicleParams::Differential>(m, "Differential")
        .value("Open", vdsim::VehicleParams::Differential::Open)
        .value("Locked", vdsim::VehicleParams::Differential::Locked)
        .value("LSD", vdsim::VehicleParams::Differential::LSD)
        .export_values();
    py::enum_<vdsim::SolverParams::Integrator>(m, "Integrator")
        .value("Euler", vdsim::SolverParams::Integrator::Euler)
        .value("RK4",   vdsim::SolverParams::Integrator::RK4)
        .export_values();
    py::enum_<vdsim::IVehicleDynamics::Level>(m, "Level")
        .value("L1_Bicycle",     vdsim::IVehicleDynamics::Level::L1_Bicycle)
        .value("L2_SevenDOF",    vdsim::IVehicleDynamics::Level::L2_SevenDOF)
        .value("L3_FourteenDOF", vdsim::IVehicleDynamics::Level::L3_FourteenDOF)
        .value("L4_Kinematic",   vdsim::IVehicleDynamics::Level::L4_Kinematic)
        .value("Lk_Kinematic",   vdsim::IVehicleDynamics::Level::Lk_Kinematic)
        .export_values();

    // -------- VehicleParams --------
    py::class_<vdsim::VehicleParams>(m, "VehicleParams")
        .def(py::init<>())
        .def_readwrite("mass",                 &vdsim::VehicleParams::mass)
        .def_readwrite("mass_sprung",          &vdsim::VehicleParams::mass_sprung)
        .def_readwrite("wheelbase",            &vdsim::VehicleParams::wheelbase)
        .def_readwrite("cg_to_front",          &vdsim::VehicleParams::cg_to_front)
        .def_readwrite("cg_to_rear",           &vdsim::VehicleParams::cg_to_rear)
        .def_readwrite("track_front",          &vdsim::VehicleParams::track_front)
        .def_readwrite("track_rear",           &vdsim::VehicleParams::track_rear)
        .def_readwrite("cg_height",            &vdsim::VehicleParams::cg_height)
        .def_readwrite("wheel_radius_nominal", &vdsim::VehicleParams::wheel_radius_nominal)
        .def_readwrite("arb_stiffness_front",  &vdsim::VehicleParams::arb_stiffness_front)
        .def_readwrite("arb_stiffness_rear",   &vdsim::VehicleParams::arb_stiffness_rear)
        .def_readwrite("roll_center_height_front", &vdsim::VehicleParams::roll_center_height_front)
        .def_readwrite("roll_center_height_rear",  &vdsim::VehicleParams::roll_center_height_rear)
        // Sprung-mass inertia tensor diagonal (Ixx roll, Iyy pitch, Izz yaw).
        .def_property("ixx",
            [](const vdsim::VehicleParams& p) { return p.inertia_diag.x(); },
            [](vdsim::VehicleParams& p, double v) { p.inertia_diag.x() = v; })
        .def_property("iyy",
            [](const vdsim::VehicleParams& p) { return p.inertia_diag.y(); },
            [](vdsim::VehicleParams& p, double v) { p.inertia_diag.y() = v; })
        .def_property("izz",
            [](const vdsim::VehicleParams& p) { return p.inertia_diag.z(); },
            [](vdsim::VehicleParams& p, double v) { p.inertia_diag.z() = v; })
        .def_readwrite("spring_stiffness",     &vdsim::VehicleParams::spring_stiffness)
        .def_readwrite("damper_coefficient",   &vdsim::VehicleParams::damper_coefficient)
        .def_readwrite("unsprung_mass",        &vdsim::VehicleParams::unsprung_mass)
        .def_readwrite("wheel_inertia",        &vdsim::VehicleParams::wheel_inertia)
        .def_readwrite("camber_per_roll",      &vdsim::VehicleParams::camber_per_roll)
        .def_readwrite("drive_type",           &vdsim::VehicleParams::drive_type)
        .def_readwrite("drive_split_front",    &vdsim::VehicleParams::drive_split_front)
        .def_readwrite("plant_path",           &vdsim::VehicleParams::plant_path)
        .def_readwrite("differential",         &vdsim::VehicleParams::differential)
        .def_readwrite("lsd_preload",          &vdsim::VehicleParams::lsd_preload)
        .def_readwrite("lsd_ramp",             &vdsim::VehicleParams::lsd_ramp)
        .def_readwrite("max_motor_torque",     &vdsim::VehicleParams::max_motor_torque)
        .def_readwrite("final_drive_ratio",    &vdsim::VehicleParams::final_drive_ratio)
        .def_readwrite("engine_rotational_inertia",
                       &vdsim::VehicleParams::engine_rotational_inertia)
        .def_readwrite("max_brake_torque",     &vdsim::VehicleParams::max_brake_torque)
        .def_readwrite("brake_bias_front",     &vdsim::VehicleParams::brake_bias_front)
        .def_readwrite("brake_ebd_enabled",    &vdsim::VehicleParams::brake_ebd_enabled)
        .def_readwrite("brake_deadtime_s",     &vdsim::VehicleParams::brake_deadtime_s)
        .def_readwrite("drive_deadtime_s",     &vdsim::VehicleParams::drive_deadtime_s)
        .def_readwrite("steer_deadtime_s",     &vdsim::VehicleParams::steer_deadtime_s)
        .def_readwrite("steering_ratio",       &vdsim::VehicleParams::steering_ratio)
        .def_readwrite("max_steer_angle_wheel",&vdsim::VehicleParams::max_steer_angle_wheel)
        .def_readwrite("ackerman_percent",     &vdsim::VehicleParams::ackerman_percent)
        .def_readwrite("steering_dynamic",     &vdsim::VehicleParams::steering_dynamic)
        .def_readwrite("rack_mass",            &vdsim::VehicleParams::rack_mass)
        .def_readwrite("rack_damping",         &vdsim::VehicleParams::rack_damping)
        .def_readwrite("pinion_radius",        &vdsim::VehicleParams::pinion_radius)
        .def_readwrite("caster_trail",         &vdsim::VehicleParams::caster_trail)
        .def_readwrite("aero_drag_coeff",      &vdsim::VehicleParams::aero_drag_coeff)
        .def_readwrite("frontal_area",         &vdsim::VehicleParams::frontal_area)
        .def_readwrite("aero_lift_front",      &vdsim::VehicleParams::aero_lift_front)
        .def_readwrite("aero_lift_rear",       &vdsim::VehicleParams::aero_lift_rear)
        .def_readwrite("anti_dive_front",      &vdsim::VehicleParams::anti_dive_front)
        .def_readwrite("anti_squat_rear",      &vdsim::VehicleParams::anti_squat_rear)
        .def_static("from_yaml", &vdsim::VehicleParams::from_yaml)
        .def("to_yaml",          &vdsim::VehicleParams::to_yaml);

    py::class_<vdsim::LuGreTireParams>(m, "LuGreTireParams")
        .def(py::init<>())
        .def_readwrite("enabled", &vdsim::LuGreTireParams::enabled)
        .def_readwrite("sigma0",  &vdsim::LuGreTireParams::sigma0)
        .def_readwrite("sigma1",  &vdsim::LuGreTireParams::sigma1)
        .def_readwrite("sigma2",  &vdsim::LuGreTireParams::sigma2)
        .def_readwrite("m_eff",   &vdsim::LuGreTireParams::m_eff);

    // -------- TireParams --------
    py::class_<vdsim::TireParams>(m, "TireParams")
        .def(py::init<>())
        .def_readwrite("B_long", &vdsim::TireParams::B_long)
        .def_readwrite("C_long", &vdsim::TireParams::C_long)
        .def_readwrite("D_long", &vdsim::TireParams::D_long)
        .def_readwrite("E_long", &vdsim::TireParams::E_long)
        .def_readwrite("B_lat",  &vdsim::TireParams::B_lat)
        .def_readwrite("C_lat",  &vdsim::TireParams::C_lat)
        .def_readwrite("D_lat",  &vdsim::TireParams::D_lat)
        .def_readwrite("E_lat",  &vdsim::TireParams::E_lat)
        .def_readwrite("mu_nominal",          &vdsim::TireParams::mu_nominal)
        .def_readwrite("Fz_nominal",          &vdsim::TireParams::Fz_nominal)
        .def_readwrite("cornering_stiffness", &vdsim::TireParams::cornering_stiffness)
        .def_readwrite("rolling_resistance",  &vdsim::TireParams::rolling_resistance)
        .def_readwrite("combined_slip_enabled", &vdsim::TireParams::combined_slip_enabled)
        .def_readwrite("pneumatic_trail",     &vdsim::TireParams::pneumatic_trail)
        .def_readwrite("trail_falloff_alpha", &vdsim::TireParams::trail_falloff_alpha)
        .def_readwrite("camber_stiffness",    &vdsim::TireParams::camber_stiffness)
        .def_readwrite("load_sensitivity",    &vdsim::TireParams::load_sensitivity)
        .def_readwrite("relaxation_length_lat",  &vdsim::TireParams::relaxation_length_lat)
        .def_readwrite("relaxation_length_long", &vdsim::TireParams::relaxation_length_long)
        .def_readwrite("tire_vertical_stiffness", &vdsim::TireParams::tire_vertical_stiffness)
        .def_readwrite("reff_breff", &vdsim::TireParams::reff_breff)
        .def_readwrite("reff_dreff", &vdsim::TireParams::reff_dreff)
        .def_readwrite("reff_freff", &vdsim::TireParams::reff_freff)
        .def_readwrite("crown_radius", &vdsim::TireParams::crown_radius)
        .def_readwrite("backend", &vdsim::TireParams::backend)
        .def_readwrite("tir_path", &vdsim::TireParams::tir_path)
        .def_readwrite("lugre", &vdsim::TireParams::lugre)
        .def_static("from_yaml", &vdsim::TireParams::from_yaml)
        .def("to_yaml",          &vdsim::TireParams::to_yaml);

    // -------- ITireModel (stand-alone tire query) --------
    py::class_<vdsim::ITireModel::Input>(m, "TireInput")
        .def(py::init<>())
        .def_readwrite("Fz",       &vdsim::ITireModel::Input::Fz)
        .def_readwrite("kappa",    &vdsim::ITireModel::Input::kappa)
        .def_readwrite("alpha",    &vdsim::ITireModel::Input::alpha)
        .def_readwrite("mu_long",  &vdsim::ITireModel::Input::mu_long)
        .def_readwrite("mu_lat",   &vdsim::ITireModel::Input::mu_lat)
        .def_readwrite("Vx_wheel", &vdsim::ITireModel::Input::Vx_wheel)
        .def_readwrite("gamma",    &vdsim::ITireModel::Input::gamma);
    py::class_<vdsim::ITireModel::Output>(m, "TireOutput")
        .def_readonly("Fx", &vdsim::ITireModel::Output::Fx)
        .def_readonly("Fy", &vdsim::ITireModel::Output::Fy)
        .def_readonly("Mz", &vdsim::ITireModel::Output::Mz);
    py::class_<vdsim::ITireModel>(m, "ITireModel")
        .def("initialize", &vdsim::ITireModel::initialize)
        .def("compute",    &vdsim::ITireModel::compute);
    m.def("create_pacejka_mf96", &vdsim::create_pacejka_mf96);
    m.def("create_linear_tire",  &vdsim::create_linear_tire);
    m.def("create_magic_formula_tire_from_tir",
          &vdsim::create_magic_formula_tire_from_tir,
          py::arg("tir_path"),
          "Load MF2002 tire evaluator from a .tir file. Returns ITireModel.");

    // -------- ISuspensionKinematics (Ld4) --------
    py::class_<vdsim::ISuspensionKinematics::Output>(m, "KinematicsOutput")
        .def_readonly("camber",       &vdsim::ISuspensionKinematics::Output::camber)
        .def_readonly("toe",          &vdsim::ISuspensionKinematics::Output::toe)
        .def_readonly("track_change", &vdsim::ISuspensionKinematics::Output::track_change)
        .def_readonly("caster",       &vdsim::ISuspensionKinematics::Output::caster);
    py::class_<vdsim::ISuspensionKinematics>(m, "ISuspensionKinematics")
        .def("compute", &vdsim::ISuspensionKinematics::compute,
             py::arg("wheel_travel"), py::arg("steer_input") = 0.0);
    m.def("create_lookup_kinematics",
          [](const std::string& csv_path) {
              return vdsim::create_lookup_kinematics(csv_path);
          });
    m.def("create_dw_native_kinematics",
          [](const std::string& yaml_path) {
              return vdsim::create_dw_native_kinematics(yaml_path);
          });
    m.def("create_ta_native_kinematics",
          [](const std::string& yaml_path) {
              return vdsim::create_ta_native_kinematics(yaml_path);
          });
    m.def("create_mp_native_kinematics",
          [](const std::string& yaml_path) {
              return vdsim::create_mp_native_kinematics(yaml_path);
          });
    m.def("create_5link_native_kinematics",
          [](const std::string& yaml_path) {
              return vdsim::create_5link_native_kinematics(yaml_path);
          });
    m.def("attach_front_kinematics",
          [](vdsim::IVehicleDynamics& dyn, const std::string& csv_path) {
              auto k = vdsim::create_lookup_kinematics(csv_path);
              return vdsim::attach_front_kinematics(dyn, std::move(k));
          });
    m.def("attach_rear_kinematics",
          [](vdsim::IVehicleDynamics& dyn, const std::string& csv_path) {
              auto k = vdsim::create_lookup_kinematics(csv_path);
              return vdsim::attach_rear_kinematics(dyn, std::move(k));
          });

    py::class_<vdsim::mb::KcSweepSample>(m, "KcSweepSample")
        .def_readonly("abscissa", &vdsim::mb::KcSweepSample::abscissa)
        .def_readonly("toe_deg", &vdsim::mb::KcSweepSample::toe_deg)
        .def_readonly("camber_deg", &vdsim::mb::KcSweepSample::camber_deg)
        .def_readonly("caster_deg", &vdsim::mb::KcSweepSample::caster_deg)
        .def_readonly("track_mm", &vdsim::mb::KcSweepSample::track_mm)
        .def_readonly("compliance_toe_deg",
                       &vdsim::mb::KcSweepSample::compliance_toe_deg)
        .def_readonly("compliance_camber_deg",
                       &vdsim::mb::KcSweepSample::compliance_camber_deg);
    py::class_<vdsim::mb::KcSweepResult>(m, "KcSweepResult")
        .def_readonly("travel", &vdsim::mb::KcSweepResult::travel)
        .def_readonly("steer", &vdsim::mb::KcSweepResult::steer)
        .def_readonly("compliance_fy", &vdsim::mb::KcSweepResult::compliance_fy);
    m.def("run_kc_sweep",
          [](const std::string& kin_yaml_path) {
              auto topo = vdsim::mb::SuspensionTopology::from_yaml(kin_yaml_path);
              return vdsim::mb::run_kc_sweep(topo);
          },
          py::arg("kin_yaml_path"));
    py::class_<vdsim::mb::KcMetrics>(m, "KcMetrics")
        .def_readonly("toe_gain_travel_deg_per_mm",
                       &vdsim::mb::KcMetrics::toe_gain_travel_deg_per_mm)
        .def_readonly("camber_gain_travel_deg_per_mm",
                       &vdsim::mb::KcMetrics::camber_gain_travel_deg_per_mm)
        .def_readonly("track_gain_travel_mm_per_mm",
                       &vdsim::mb::KcMetrics::track_gain_travel_mm_per_mm)
        .def_readonly("toe_gain_steer_deg_per_mm",
                       &vdsim::mb::KcMetrics::toe_gain_steer_deg_per_mm)
        .def_readonly("caster_gain_steer_deg_per_mm",
                       &vdsim::mb::KcMetrics::caster_gain_steer_deg_per_mm);
    py::class_<vdsim::mb::KcMetricDelta>(m, "KcMetricDelta")
        .def_readonly("name", &vdsim::mb::KcMetricDelta::name)
        .def_readonly("reference", &vdsim::mb::KcMetricDelta::reference)
        .def_readonly("candidate", &vdsim::mb::KcMetricDelta::candidate)
        .def_readonly("rel_error", &vdsim::mb::KcMetricDelta::rel_error)
        .def_readonly("ok", &vdsim::mb::KcMetricDelta::ok);
    py::class_<vdsim::mb::KcXcheckReport>(m, "KcXcheckReport")
        .def_readonly("all_ok", &vdsim::mb::KcXcheckReport::all_ok)
        .def_readonly("deltas", &vdsim::mb::KcXcheckReport::deltas);
    m.def("compute_kc_metrics", &vdsim::mb::compute_kc_metrics);
    m.def("compare_kc_metrics", &vdsim::mb::compare_kc_metrics,
          py::arg("reference"), py::arg("candidate"), py::arg("rtol") = 0.05,
          py::arg("atol") = 1e-4);
    m.def("run_kc_xcheck",
          [](const std::string& reference_yaml, const std::string& candidate_yaml,
             double rtol) {
              return vdsim::mb::run_kc_xcheck(reference_yaml, candidate_yaml, rtol);
          },
          py::arg("reference_yaml"), py::arg("candidate_yaml"),
          py::arg("rtol") = 0.05);

    // -------- SolverParams --------
    py::class_<vdsim::SolverParams>(m, "SolverParams")
        .def(py::init<>())
        .def_readwrite("integrator",     &vdsim::SolverParams::integrator)
        .def_readwrite("max_substep_dt", &vdsim::SolverParams::max_substep_dt)
        .def_readwrite("max_substeps",   &vdsim::SolverParams::max_substeps)
        .def_static("from_yaml", &vdsim::SolverParams::from_yaml)
        .def("to_yaml",          &vdsim::SolverParams::to_yaml);

    // -------- State --------
    py::class_<vdsim::State>(m, "State")
        .def(py::init<>())
        .def_readwrite("position",         &vdsim::State::position)
        .def_readwrite("orientation",      &vdsim::State::orientation)
        .def_readwrite("velocity",         &vdsim::State::velocity)
        .def_readwrite("angular_velocity", &vdsim::State::angular_velocity)
        .def_readwrite("wheel_spin",       &vdsim::State::wheel_spin)
        .def_readwrite("susp_compression", &vdsim::State::susp_compression)
        .def_readwrite("susp_velocity",    &vdsim::State::susp_velocity)
        .def_readwrite("unsprung_pos",     &vdsim::State::unsprung_pos)
        .def_readwrite("unsprung_vel",     &vdsim::State::unsprung_vel)
        .def_readwrite("rack_travel",      &vdsim::State::rack_travel)
        .def_readwrite("rack_velocity",    &vdsim::State::rack_velocity)
        .def("yaw",        &vdsim::State::yaw)
        .def("yaw_rate",   &vdsim::State::yaw_rate)
        .def("vx",         &vdsim::State::vx)
        .def("vy",         &vdsim::State::vy)
        .def("beta",       &vdsim::State::beta);

    // Build an initial State from pose + speed (handles the yaw->quaternion).
    m.def("make_init_state",
          [](double x, double y, double yaw, double v, double wheel_radius) {
              vdsim::State s;
              s.position = vdsim::Vec3(x, y, 0.0);
              s.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, yaw});
              s.velocity = vdsim::Vec3(v, 0.0, 0.0);
              const double w = (wheel_radius > 1e-6) ? v / wheel_radius : 0.0;
              s.wheel_spin = {{w, w, w, w}};
              return s;
          },
          py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("yaw") = 0.0,
          py::arg("v") = 0.0, py::arg("wheel_radius") = 0.32);

    // Re-consistent overload: per-wheel wheel_spin = v/Re(static Fz) so a free-rolling
    // tire reports slip=0 (matches MF-Tyre/CarMaker init). Identical to the radius
    // overload when the tire has no Re coefficients (reff_*=0).
    m.def("make_init_state",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             double x, double y, double yaw, double v) {
              vdsim::State s;
              s.position = vdsim::Vec3(x, y, 0.0);
              s.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, yaw});
              s.velocity = vdsim::Vec3(v, 0.0, 0.0);
              s.wheel_spin = vdsim::free_roll_wheel_spin(vp, tp, v);
              return s;
          },
          py::arg("vp"), py::arg("tp"), py::arg("x") = 0.0, py::arg("y") = 0.0,
          py::arg("yaw") = 0.0, py::arg("v") = 0.0);

    m.def("make_init_state_6dof",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             double x, double y, double yaw, double vx, double vy, double r) {
              vdsim::State s;
              s.position = vdsim::Vec3(x, y, 0.0);
              s.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, yaw});
              s.velocity = vdsim::Vec3(vx, vy, 0.0);
              s.angular_velocity = vdsim::Vec3(0.0, 0.0, r);
              s.wheel_spin = vdsim::free_roll_wheel_spin(vp, tp, vx);
              return s;
          },
          py::arg("vp"), py::arg("tp"),
          py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("yaw") = 0.0,
          py::arg("vx") = 0.0, py::arg("vy") = 0.0, py::arg("r") = 0.0);

    m.def("free_roll_wheel_spin",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp, double vx) {
              const auto w = vdsim::free_roll_wheel_spin(vp, tp, vx);
              return std::vector<double>(w.begin(), w.end());
          },
          py::arg("vp"), py::arg("tp"), py::arg("vx"),
          "Per-wheel free-rolling wheel spin [rad/s] (FL,FR,RL,RR) using Re(Fz).");

    // Linearize the planar vehicle map about an operating point via central
    // finite differences. State x=[X,Y,psi,vx,vy,r], input u=[steer, pedal]
    // (pedal>0 -> throttle, <0 -> brake). Returns the discrete one-step Jacobian
    // (A,B) at dt: x_{k+1} ~= A x_k + B u_k around the trim. Continuous-time
    // approx: A_c=(A-I)/dt, B_c=B/dt.
    m.def("linearize",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             const std::string& level, std::vector<double> x0,
             std::vector<double> u0, double dt, double mu) {
              auto make_dyn = [&]() -> std::unique_ptr<vdsim::IVehicleDynamics> {
                  if (level == "K" || level == "L0") return vdsim::create_kinematic();
                  if (level == "L1") return vdsim::create_bicycle();
                  if (level == "L3") return vdsim::create_fourteen_dof();
                  if (level == "L4") return vdsim::create_fourteen_dof_kinematic();
                  return vdsim::create_seven_dof();
              };
              auto dyn = make_dyn();
              vdsim::SolverParams sp;
              dyn->initialize(vp, tp, sp);
              auto ground = vdsim::create_flat_ground(0.0, mu);
              x0.resize(6, 0.0); u0.resize(2, 0.0);

              auto f = [&](std::vector<double> x, std::vector<double> u) {
                  vdsim::State s;
                  s.position = vdsim::Vec3(x[0], x[1], 0.0);
                  s.orientation = vdsim::quat_from_euler(vdsim::Euler{0.0, 0.0, x[2]});
                  s.velocity = vdsim::Vec3(x[3], x[4], 0.0);
                  s.angular_velocity = vdsim::Vec3(0.0, 0.0, x[5]);
                  s.wheel_spin = vdsim::free_roll_wheel_spin(vp, tp, x[3]);
                  dyn->reset(s);
                  vdsim::CmdL4 c; c.steer_angle_wheel = u[0];
                  if (u[1] >= 0.0) { c.throttle = u[1]; } else { c.brake = -u[1]; }
                  vdsim::ContactArray contacts;
                  ground->query(dyn->state(), vp, contacts);
                  dyn->step(vdsim::ControlInput(c), contacts, dt);
                  const vdsim::State& o = dyn->state();
                  return std::vector<double>{o.position.x(), o.position.y(),
                      vdsim::yaw_from_quat(o.orientation), o.velocity.x(),
                      o.velocity.y(), o.angular_velocity.z()};
              };

              const std::vector<double> ex{0.5, 0.5, 0.01, 0.1, 0.1, 0.02};
              const std::vector<double> eu{0.002, 0.02};
              std::vector<std::vector<double>> A(6, std::vector<double>(6, 0.0));
              std::vector<std::vector<double>> B(6, std::vector<double>(2, 0.0));
              for (int j = 0; j < 6; ++j) {
                  auto xp = x0, xm = x0; xp[j] += ex[j]; xm[j] -= ex[j];
                  const auto fp = f(xp, u0), fm = f(xm, u0);
                  for (int i = 0; i < 6; ++i) A[i][j] = (fp[i] - fm[i]) / (2.0 * ex[j]);
              }
              for (int j = 0; j < 2; ++j) {
                  auto up = u0, um = u0; up[j] += eu[j]; um[j] -= eu[j];
                  const auto fp = f(x0, up), fm = f(x0, um);
                  for (int i = 0; i < 6; ++i) B[i][j] = (fp[i] - fm[i]) / (2.0 * eu[j]);
              }
              py::dict r;
              r["A"] = A; r["B"] = B; r["dt"] = dt;
              r["state_names"] = std::vector<std::string>{"X","Y","psi","vx","vy","r"};
              r["input_names"] = std::vector<std::string>{"steer","pedal"};
              return r;
          },
          py::arg("vehicle"), py::arg("tire"), py::arg("level") = "L2",
          py::arg("x0"), py::arg("u0"), py::arg("dt") = 0.01, py::arg("mu") = 1.0);

    // -------- ContactPoint --------
    py::class_<vdsim::ContactPoint>(m, "ContactPoint")
        .def(py::init<>())
        .def_readwrite("position",  &vdsim::ContactPoint::position)
        .def_readwrite("normal",    &vdsim::ContactPoint::normal)
        .def_readwrite("is_valid",  &vdsim::ContactPoint::is_valid)
        .def_readwrite("mu_long",   &vdsim::ContactPoint::mu_long)
        .def_readwrite("mu_lat",    &vdsim::ContactPoint::mu_lat)
        .def_readwrite("surface_id", &vdsim::ContactPoint::surface_id);

    // -------- CmdL1 / CmdL3 (VLA plant direct torque path) --------
    py::class_<vdsim::CmdL1>(m, "CmdL1")
        .def(py::init<>())
        .def_readwrite("motor_torque",      &vdsim::CmdL1::motor_torque)
        .def_readwrite("brake_torque",      &vdsim::CmdL1::brake_torque)
        .def_readwrite("steer_angle_wheel", &vdsim::CmdL1::steer_angle_wheel);
    py::class_<vdsim::CmdL3>(m, "CmdL3")
        .def(py::init<>())
        .def_readwrite("Fx_total",          &vdsim::CmdL3::Fx_total)
        .def_readwrite("steer_angle_wheel", &vdsim::CmdL3::steer_angle_wheel);

    // -------- CmdL4 (used by step()) --------
    py::class_<vdsim::CmdL4>(m, "CmdL4")
        .def(py::init<>())
        .def_readwrite("throttle",          &vdsim::CmdL4::throttle)
        .def_readwrite("brake",             &vdsim::CmdL4::brake)
        .def_readwrite("steer_angle_wheel", &vdsim::CmdL4::steer_angle_wheel)
        .def_readwrite("gear",              &vdsim::CmdL4::gear);

    // -------- Higher control-ladder commands (cascaded by SimSession) --------
    // L5: longitudinal accel target + steer angle.
    py::class_<vdsim::CmdL5>(m, "CmdL5")
        .def(py::init<>())
        .def_readwrite("ax_target",         &vdsim::CmdL5::ax_target)
        .def_readwrite("steer_angle_wheel", &vdsim::CmdL5::steer_angle_wheel);
    // L6: longitudinal speed target + steer angle.
    py::class_<vdsim::CmdL6>(m, "CmdL6")
        .def(py::init<>())
        .def_readwrite("v_target",          &vdsim::CmdL6::v_target)
        .def_readwrite("steer_angle_wheel", &vdsim::CmdL6::steer_angle_wheel);
    // L7: speed target + path curvature.
    py::class_<vdsim::CmdL7>(m, "CmdL7")
        .def(py::init<>())
        .def_readwrite("v_target", &vdsim::CmdL7::v_target)
        .def_readwrite("kappa",    &vdsim::CmdL7::kappa);

    // ---- Split lon/lat commands (independent axis levels) ----
    // Longitudinal axis levels.
    py::class_<vdsim::LcLonL4>(m, "LcLonL4")
        .def(py::init<>())
        .def_readwrite("throttle", &vdsim::LcLonL4::throttle)
        .def_readwrite("brake",    &vdsim::LcLonL4::brake)
        .def_readwrite("gear",     &vdsim::LcLonL4::gear);
    py::class_<vdsim::LcLonL5>(m, "LcLonL5")
        .def(py::init<>()).def_readwrite("ax_target", &vdsim::LcLonL5::ax_target);
    py::class_<vdsim::LcLonL6>(m, "LcLonL6")
        .def(py::init<>()).def_readwrite("vx_target", &vdsim::LcLonL6::vx_target);
    // Lateral axis levels (L1-L3 need a Dynamic steering subsystem).
    py::class_<vdsim::LcLatL1>(m, "LcLatL1")
        .def(py::init<>()).def_readwrite("steer_torque", &vdsim::LcLatL1::steer_torque);
    py::class_<vdsim::LcLatL2>(m, "LcLatL2")
        .def(py::init<>()).def_readwrite("steer_ang_accel", &vdsim::LcLatL2::steer_ang_accel);
    py::class_<vdsim::LcLatL3>(m, "LcLatL3")
        .def(py::init<>()).def_readwrite("steer_ang_vel", &vdsim::LcLatL3::steer_ang_vel);
    py::class_<vdsim::LcLatL4>(m, "LcLatL4")
        .def(py::init<>()).def_readwrite("steer_angle", &vdsim::LcLatL4::steer_angle);
    py::class_<vdsim::LcLatL5>(m, "LcLatL5")
        .def(py::init<>()).def_readwrite("ay_target", &vdsim::LcLatL5::ay_target);
    py::class_<vdsim::LcLatL6>(m, "LcLatL6")
        .def(py::init<>()).def_readwrite("r_target", &vdsim::LcLatL6::r_target);
    py::class_<vdsim::LcLatL7>(m, "LcLatL7")
        .def(py::init<>()).def_readwrite("kappa", &vdsim::LcLatL7::kappa);
    // CmdSplit: independent lon + lat. lon/lat are std::variant fields; the
    // <stl.h> variant caster accepts any registered LcLon*/LcLat* python object.
    py::class_<vdsim::CmdSplit>(m, "CmdSplit")
        .def(py::init<>())
        .def_readwrite("lon", &vdsim::CmdSplit::lon)
        .def_readwrite("lat", &vdsim::CmdSplit::lat);

    // ControlInput is std::variant<CmdL1..CmdL8, CmdSplit>; pybind11's <stl.h>
    // variant caster converts any registered Cmd python object → ControlInput, so
    // sim.set_input(CmdL6(...)) / sim.set_input(CmdSplit(...)) both dispatch.

    // -------- IVehicleDynamics --------
    py::class_<vdsim::ShiftContext>(m, "ShiftContext")
        .def_readonly("engine_rpm",    &vdsim::ShiftContext::engine_rpm)
        .def_readonly("current_gear",  &vdsim::ShiftContext::current_gear)
        .def_readonly("vehicle_speed", &vdsim::ShiftContext::vehicle_speed)
        .def_readonly("throttle",      &vdsim::ShiftContext::throttle)
        .def_readonly("brake",         &vdsim::ShiftContext::brake)
        .def_readonly("num_gears",     &vdsim::ShiftContext::num_gears);

    // -------- User-defined subsystem modules --------
    // I/O structs passed to/from a user module's callbacks.
    py::class_<vdsim::DriverCmd>(m, "DriverCmd")
        .def(py::init<>())
        .def_readwrite("handwheel_angle", &vdsim::DriverCmd::handwheel_angle)
        .def_readwrite("throttle",        &vdsim::DriverCmd::throttle)
        .def_readwrite("brake",           &vdsim::DriverCmd::brake)
        .def_readwrite("gear",            &vdsim::DriverCmd::gear)
        .def_readwrite("handbrake",       &vdsim::DriverCmd::handbrake);
    py::class_<vdsim::SubsystemContext>(m, "SubsystemContext")
        .def_property_readonly("state",
            [](const vdsim::SubsystemContext& c) -> const vdsim::State& { return c.state; },
            py::return_value_policy::reference_internal)
        .def_property_readonly("cmd",
            [](const vdsim::SubsystemContext& c) -> const vdsim::DriverCmd& { return c.cmd; },
            py::return_value_policy::reference_internal)
        .def_readonly("dt", &vdsim::SubsystemContext::dt)
        .def_readonly("Fz", &vdsim::SubsystemContext::Fz);
    py::class_<vdsim::SteeringOutput>(m, "SteeringOutput")
        .def(py::init<>())
        .def_readwrite("roadwheel_angle", &vdsim::SteeringOutput::roadwheel_angle)
        .def_readwrite("rack_travel",     &vdsim::SteeringOutput::rack_travel);
    py::class_<vdsim::DrivetrainOutput>(m, "DrivetrainOutput")
        .def(py::init<>())
        .def_readwrite("wheel_torque", &vdsim::DrivetrainOutput::wheel_torque);
    py::class_<vdsim::CornerInput>(m, "CornerInput")
        .def(py::init<>())
        .def_readwrite("corner",        &vdsim::CornerInput::corner)
        .def_readwrite("defl",          &vdsim::CornerInput::defl)
        .def_readwrite("defl_rate",     &vdsim::CornerInput::defl_rate)
        .def_readwrite("damping_scale", &vdsim::CornerInput::damping_scale);
    py::class_<vdsim::AxleDefl>(m, "AxleDefl")
        .def(py::init<>())
        .def_readwrite("defl_left",  &vdsim::AxleDefl::defl_left)
        .def_readwrite("defl_right", &vdsim::AxleDefl::defl_right)
        .def_readwrite("rate_left",  &vdsim::AxleDefl::rate_left)
        .def_readwrite("rate_right", &vdsim::AxleDefl::rate_right);

    // Subclassable module bases. Override the listed method (begin_step/reset
    // optional) in a Python subclass, then install with model.set_*_module(obj).
    py::class_<vdsim::IBrakeSystem, PyBrakeSystem, std::shared_ptr<vdsim::IBrakeSystem>>(
            m, "BrakeModule",
            "Subclass and override wheel_torque(ctx)->[4] (per-wheel brake torque, N m). "
            "begin_step(ctx, dt) runs once per step; wheel_torque() per RK4 stage.")
        .def(py::init<>())
        .def("wheel_torque", &vdsim::IBrakeSystem::wheel_torque)
        .def("begin_step",   &vdsim::IBrakeSystem::begin_step)
        .def("reset",        &vdsim::IBrakeSystem::reset);
    // SimpleABS reference module: dyn.set_brake_module(vdsim.make_simple_abs(vp)).
    m.def("make_simple_abs",
          [](const vdsim::VehicleParams& vp, double slip_target, double slip_release) {
              return std::shared_ptr<vdsim::IBrakeSystem>(
                  std::make_shared<vdsim::SimpleABS>(vp, slip_target, slip_release));
          },
          py::arg("vp"), py::arg("slip_target") = 0.12, py::arg("slip_release") = 0.18,
          "Reference anti-lock brake module; install via dyn.set_brake_module(...).");
    py::class_<vdsim::ISteeringSystem, PySteeringSystem, std::shared_ptr<vdsim::ISteeringSystem>>(
            m, "SteeringModule",
            "Subclass and override apply(ctx)->SteeringOutput (roadwheel_angle, rack_travel).")
        .def(py::init<>())
        .def("apply",      &vdsim::ISteeringSystem::apply)
        .def("begin_step", &vdsim::ISteeringSystem::begin_step)
        .def("reset",      &vdsim::ISteeringSystem::reset);
    py::class_<vdsim::IDrivetrain, PyDrivetrain, std::shared_ptr<vdsim::IDrivetrain>>(
            m, "DrivetrainModule",
            "Subclass and override apply(ctx)->DrivetrainOutput. Optionally override "
            "engine_rpm()/current_gear() for telemetry.")
        .def(py::init<>())
        .def("apply",        &vdsim::IDrivetrain::apply)
        .def("begin_step",   &vdsim::IDrivetrain::begin_step)
        .def("reset",        &vdsim::IDrivetrain::reset)
        .def("engine_rpm",   &vdsim::IDrivetrain::engine_rpm)
        .def("current_gear", &vdsim::IDrivetrain::current_gear);
    py::class_<vdsim::ISuspension, PySuspension, std::shared_ptr<vdsim::ISuspension>>(
            m, "SuspensionModule",
            "Subclass and override force(ctx, CornerInput)->double (corner force, N). "
            "Evaluated per RK4 stage; must be memoryless (a function of the deflection).")
        .def(py::init<>())
        .def("force", &vdsim::ISuspension::force)
        .def("reset", &vdsim::ISuspension::reset);
    py::class_<vdsim::IAntiRollBar, PyAntiRollBar, std::shared_ptr<vdsim::IAntiRollBar>>(
            m, "AntiRollBarModule",
            "Subclass and override force(ctx, AxleDefl)->(left, right) wheel forces, N. "
            "Evaluated per RK4 stage; must be memoryless.")
        .def(py::init<>())
        .def("force", &vdsim::IAntiRollBar::force)
        .def("reset", &vdsim::IAntiRollBar::reset);

    py::class_<vdsim::IVehicleDynamics>(m, "IVehicleDynamics")
        .def("level", &vdsim::IVehicleDynamics::level)
        .def("initialize", &vdsim::IVehicleDynamics::initialize)
        .def("reset", &vdsim::IVehicleDynamics::reset)
        .def("step",
             [](vdsim::IVehicleDynamics& self,
                const vdsim::CmdL4& cmd,
                const vdsim::ContactArray& contacts, double dt) {
                 self.step(vdsim::ControlInput(cmd), contacts, dt);
             })
        .def("state", &vdsim::IVehicleDynamics::state,
             py::return_value_policy::reference_internal)
        .def("tire_Fz",  &vdsim::IVehicleDynamics::tire_Fz)
        .def("tire_forces_body", &vdsim::IVehicleDynamics::tire_forces_body)
        .def("tire_forces_wheel", &vdsim::IVehicleDynamics::tire_forces_wheel)
        .def("engine_rpm",   &vdsim::IVehicleDynamics::engine_rpm)
        .def("current_gear", &vdsim::IVehicleDynamics::current_gear)
        .def("set_shift_policy", &vdsim::IVehicleDynamics::set_shift_policy,
             "Install a Python shift policy: fn(ShiftContext) -> desired gear. "
             "Returns False if this model has no gearbox.")
        .def("set_brake_module",       &vdsim::IVehicleDynamics::set_brake_module,
             "Install a user BrakeModule (L2/L3). False if unsupported.")
        .def("set_steering_module",    &vdsim::IVehicleDynamics::set_steering_module,
             "Install a user SteeringModule (L2/L3). False if unsupported.")
        .def("set_drivetrain_module",  &vdsim::IVehicleDynamics::set_drivetrain_module,
             "Install a user DrivetrainModule (L2/L3). False if unsupported.")
        .def("set_suspension_module",  &vdsim::IVehicleDynamics::set_suspension_module,
             "Install a user SuspensionModule (L3 only). False if unsupported.")
        .def("set_antirollbar_module", &vdsim::IVehicleDynamics::set_antirollbar_module,
             py::arg("axle"), py::arg("module"),
             "Install a user AntiRollBarModule on axle 0=front/1=rear (L3 only).")
        .def("wheel_slip_ratio", &vdsim::IVehicleDynamics::wheel_slip_ratio)
        .def("wheel_slip_angle", &vdsim::IVehicleDynamics::wheel_slip_angle)
        .def("wheel_mu",         &vdsim::IVehicleDynamics::wheel_mu)
        .def("wheel_mu_peak",    &vdsim::IVehicleDynamics::wheel_mu_peak)
        .def("wheel_alpha_peak", &vdsim::IVehicleDynamics::wheel_alpha_peak)
        .def("wheel_kappa_peak", &vdsim::IVehicleDynamics::wheel_kappa_peak)
        .def("wheel_overturning_moment", &vdsim::IVehicleDynamics::wheel_overturning_moment)
        .def("roll_angle_qs",  &vdsim::IVehicleDynamics::roll_angle_qs)
        .def("pitch_angle_qs", &vdsim::IVehicleDynamics::pitch_angle_qs)
        .def("ax_body_est",    &vdsim::IVehicleDynamics::ax_body_est)
        .def("ay_body_est",    &vdsim::IVehicleDynamics::ay_body_est)
        .def("set_camber_per_wheel",
             &vdsim::IVehicleDynamics::set_camber_per_wheel);

    // No-arg overloads (disambiguated from the tire-injecting overloads).
    using DynFactory = std::unique_ptr<vdsim::IVehicleDynamics> (*)();
    m.def("create_bicycle",      static_cast<DynFactory>(&vdsim::create_bicycle));
    m.def("create_seven_dof",    static_cast<DynFactory>(&vdsim::create_seven_dof));
    m.def("create_fourteen_dof", static_cast<DynFactory>(&vdsim::create_fourteen_dof));
    m.def("create_fourteen_dof_kinematic",
          static_cast<DynFactory>(&vdsim::create_fourteen_dof_kinematic));
    // Opaque holder so create_*_ground can return providers to Python and be
    // handed to make_sim_session_ground (ownership transfers on that call).
    py::class_<vdsim::IContactProvider>(m, "ContactProvider");
    m.def("create_flat_ground",  &vdsim::create_flat_ground,
          py::arg("z") = 0.0, py::arg("mu") = 1.0);
    m.def("create_split_mu_ground", &vdsim::create_split_mu_ground,
          py::arg("z") = 0.0, py::arg("mu_left") = 1.0, py::arg("mu_right") = 0.5,
          py::arg("boundary_y") = 0.0);
    m.def("create_friction_patch_ground",
          [](double z, double base_mu,
             std::vector<std::tuple<double, double, double>> patches) {
              return vdsim::create_friction_patch_ground(z, base_mu, patches);
          },
          py::arg("z") = 0.0, py::arg("base_mu") = 1.0,
          py::arg("patches") = std::vector<std::tuple<double, double, double>>{});
    m.def("create_inclined_ground", &vdsim::create_inclined_ground,
          py::arg("z0") = 0.0, py::arg("grade") = 0.0, py::arg("bank") = 0.0,
          py::arg("mu") = 1.0);
    m.def("create_rough_ground", &vdsim::create_rough_ground,
          py::arg("z") = 0.0, py::arg("mu") = 1.0, py::arg("amp") = 0.01,
          py::arg("wavelength") = 4.0);
    m.def("create_iso8608_ground", &vdsim::create_iso8608_ground,
          py::arg("z") = 0.0, py::arg("mu") = 1.0, py::arg("road_class") = 2,
          py::arg("seed") = 1u);
    m.def("create_psd_ground", &vdsim::create_psd_ground,
          py::arg("z") = 0.0, py::arg("mu") = 1.0, py::arg("gd_n0") = 256e-6,
          py::arg("waviness") = 2.0, py::arg("n_break") = 0.0,
          py::arg("waviness_high") = 2.0, py::arg("n_min") = 0.011,
          py::arg("n_max") = 4.0, py::arg("seed") = 1u);
    m.def("create_psd_ground_table", &vdsim::create_psd_ground_table,
          py::arg("z") = 0.0, py::arg("mu") = 1.0, py::arg("n") = std::vector<double>{},
          py::arg("gd") = std::vector<double>{}, py::arg("n_min") = 0.011,
          py::arg("n_max") = 10.0, py::arg("seed") = 1u);

    // Dynamics driven by a full Magic Formula tire loaded from a .tir file.
    // Keep .tir files (which may hold confidential measured data) out of the repo.
    m.def("create_bicycle_from_tir", [](const std::string& tir) {
        return vdsim::create_bicycle(vdsim::create_magic_formula_tire_from_tir(tir));
    }, py::arg("tir_path"));
    m.def("create_seven_dof_from_tir", [](const std::string& tir) {
        return vdsim::create_seven_dof(vdsim::create_magic_formula_tire_from_tir(tir));
    }, py::arg("tir_path"));
    m.def("create_fourteen_dof_from_tir", [](const std::string& tir) {
        return vdsim::create_fourteen_dof(vdsim::create_magic_formula_tire_from_tir(tir));
    }, py::arg("tir_path"));

    // -------- LongAxController / LongVxController / PurePursuit --------
    py::class_<vdsim::LongAxController::Gains>(m, "LongAxGains")
        .def(py::init<>())
        .def_readwrite("kp", &vdsim::LongAxController::Gains::kp)
        .def_readwrite("ki", &vdsim::LongAxController::Gains::ki);

    py::class_<vdsim::LongAxController>(m, "LongAxController")
        .def(py::init<>())
        .def("initialize", &vdsim::LongAxController::initialize)
        .def("reset",      &vdsim::LongAxController::reset)
        .def("update",     &vdsim::LongAxController::update);

    py::class_<vdsim::LongVxController::Gains>(m, "LongVxGains")
        .def(py::init<>())
        .def_readwrite("kp", &vdsim::LongVxController::Gains::kp)
        .def_readwrite("ki", &vdsim::LongVxController::Gains::ki);

    py::class_<vdsim::LongVxController>(m, "LongVxController")
        .def(py::init<>())
        .def("initialize", &vdsim::LongVxController::initialize)
        .def("reset",      &vdsim::LongVxController::reset)
        .def("update",     &vdsim::LongVxController::update);

    // -------- Actuator dynamics + sensor delay --------
    py::class_<vdsim::ChannelActuator>(m, "ChannelActuator")
        .def(py::init<>())
        .def_readwrite("dead_time_s", &vdsim::ChannelActuator::dead_time_s)
        .def_readwrite("tau_s",       &vdsim::ChannelActuator::tau_s)
        .def_readwrite("rate_limit",  &vdsim::ChannelActuator::rate_limit)
        .def_readwrite("dead_zone",   &vdsim::ChannelActuator::dead_zone)
        .def_readwrite("out_min",     &vdsim::ChannelActuator::out_min)
        .def_readwrite("out_max",     &vdsim::ChannelActuator::out_max);
    py::class_<vdsim::LuGreParams>(m, "LuGreParams")
        .def(py::init<>())
        .def_readwrite("enabled", &vdsim::LuGreParams::enabled)
        .def_readwrite("sigma0",  &vdsim::LuGreParams::sigma0)
        .def_readwrite("sigma1",  &vdsim::LuGreParams::sigma1)
        .def_readwrite("sigma2",  &vdsim::LuGreParams::sigma2)
        .def_readwrite("Tc",      &vdsim::LuGreParams::Tc)
        .def_readwrite("Ts",      &vdsim::LuGreParams::Ts)
        .def_readwrite("ws",      &vdsim::LuGreParams::ws);
    py::class_<vdsim::SteerActuator>(m, "SteerActuator")
        .def(py::init<>())
        .def_readwrite("ch",       &vdsim::SteerActuator::ch)
        .def_readwrite("friction", &vdsim::SteerActuator::friction)
        .def_readwrite("inertia",  &vdsim::SteerActuator::inertia)
        .def_readwrite("servo_kp", &vdsim::SteerActuator::servo_kp)
        .def_readwrite("servo_kd", &vdsim::SteerActuator::servo_kd);
    py::class_<vdsim::BrakeActuator>(m, "BrakeActuator")
        .def(py::init<>())
        .def_readwrite("ch",              &vdsim::BrakeActuator::ch)
        .def_readwrite("thermal_enabled", &vdsim::BrakeActuator::thermal_enabled)
        .def_readwrite("heat_coeff",      &vdsim::BrakeActuator::heat_coeff)
        .def_readwrite("cool_coeff",      &vdsim::BrakeActuator::cool_coeff)
        .def_readwrite("T_ambient",       &vdsim::BrakeActuator::T_ambient)
        .def_readwrite("mu_T_temp",       &vdsim::BrakeActuator::mu_T_temp)
        .def_readwrite("mu_T_scale",      &vdsim::BrakeActuator::mu_T_scale);
    py::class_<vdsim::ActuatorParams>(m, "ActuatorParams")
        .def(py::init<>())
        .def_readwrite("steer",    &vdsim::ActuatorParams::steer)
        .def_readwrite("throttle", &vdsim::ActuatorParams::throttle)
        .def_readwrite("brake",    &vdsim::ActuatorParams::brake);

    // -------- Sensor models --------
    py::class_<vdsim::SensorNoise>(m, "SensorNoise")
        .def(py::init<>())
        .def_readwrite("noise_std", &vdsim::SensorNoise::noise_std)
        .def_readwrite("bias",      &vdsim::SensorNoise::bias)
        .def_readwrite("bias_rw",   &vdsim::SensorNoise::bias_rw);
    py::class_<vdsim::SensorParams>(m, "SensorParams")
        .def(py::init<>())
        .def_readwrite("enabled",     &vdsim::SensorParams::enabled)
        .def_readwrite("seed",        &vdsim::SensorParams::seed)
        .def_readwrite("imu_accel",   &vdsim::SensorParams::imu_accel)
        .def_readwrite("imu_gyro",    &vdsim::SensorParams::imu_gyro)
        .def_readwrite("wheel_speed", &vdsim::SensorParams::wheel_speed)
        .def_readwrite("steer",       &vdsim::SensorParams::steer)
        .def_readwrite("gnss_pos",    &vdsim::SensorParams::gnss_pos)
        .def_readwrite("gnss_vel",    &vdsim::SensorParams::gnss_vel)
        .def_static("from_yaml", &vdsim::SensorParams::from_yaml, py::arg("path"))
        .def("to_yaml", &vdsim::SensorParams::to_yaml, py::arg("path"));
    py::class_<vdsim::SensorMeas>(m, "SensorMeas")
        .def(py::init<>())
        .def_readonly("ax", &vdsim::SensorMeas::ax)
        .def_readonly("ay", &vdsim::SensorMeas::ay)
        .def_readonly("az", &vdsim::SensorMeas::az)
        .def_readonly("wx", &vdsim::SensorMeas::wx)
        .def_readonly("wy", &vdsim::SensorMeas::wy)
        .def_readonly("wz", &vdsim::SensorMeas::wz)
        .def_readonly("wheel_speed", &vdsim::SensorMeas::wheel_speed)
        .def_readonly("steer",   &vdsim::SensorMeas::steer)
        .def_readonly("gnss_x",  &vdsim::SensorMeas::gnss_x)
        .def_readonly("gnss_y",  &vdsim::SensorMeas::gnss_y)
        .def_readonly("gnss_vx", &vdsim::SensorMeas::gnss_vx)
        .def_readonly("gnss_vy", &vdsim::SensorMeas::gnss_vy);

    // Step response of one actuator channel: applies a step command and returns
    // {t, cmd, out} so a GUI can plot the realized vs commanded signal.
    m.def("actuator_step_response",
          [](const vdsim::ActuatorParams& p, const std::string& channel,
             double amplitude, double dt, double duration, double speed_mps,
             double pre_s) {
              vdsim::ActuatorModel act;
              act.initialize(p, dt);
              act.reset();
              // Hold zero for pre_s before the step (t<0) so the step edge and
              // the dead-time delay are visible on the plot.
              const int n_pre = std::max(0, static_cast<int>(pre_s / dt));
              const int n     = std::max(1, static_cast<int>(duration / dt));
              std::vector<double> t, cmd, out;
              t.reserve(n_pre + n); cmd.reserve(n_pre + n); out.reserve(n_pre + n);
              for (int i = -n_pre; i < n; ++i) {
                  const double u = (i < 0) ? 0.0 : amplitude;
                  vdsim::CmdL4 d{};
                  if (channel == "steer")         d.steer_angle_wheel = u;
                  else if (channel == "throttle") d.throttle          = u;
                  else                            d.brake             = u;
                  const vdsim::CmdL4 r = act.apply(d, speed_mps, dt);
                  const double o = (channel == "steer") ? r.steer_angle_wheel
                                 : (channel == "throttle") ? r.throttle : r.brake;
                  t.push_back(i * dt); cmd.push_back(u); out.push_back(o);
              }
              py::dict res;
              res["t"] = t; res["cmd"] = cmd; res["out"] = out;
              return res;
          },
          py::arg("params"), py::arg("channel"), py::arg("amplitude") = 1.0,
          py::arg("dt") = 0.002, py::arg("duration") = 1.0,
          py::arg("speed_mps") = 15.0, py::arg("pre_s") = 0.06);

    // -------- SimSession kernel (Phase 1: web backend access) --------
    py::class_<vdsim::SimOutput>(m, "SimOutput")
        .def_readonly("state",         &vdsim::SimOutput::state)
        .def_readonly("measured",      &vdsim::SimOutput::measured)
        .def_readonly("sim_time",      &vdsim::SimOutput::sim_time)
        .def_readonly("ax",            &vdsim::SimOutput::ax)
        .def_readonly("ay",            &vdsim::SimOutput::ay)
        .def_readonly("roll",          &vdsim::SimOutput::roll)
        .def_readonly("pitch",         &vdsim::SimOutput::pitch)
        .def_readonly("Fz",            &vdsim::SimOutput::Fz)
        .def_readonly("tire_forces",   &vdsim::SimOutput::tire_forces)
        .def_readonly("slip_ratio",    &vdsim::SimOutput::slip_ratio)
        .def_readonly("slip_angle",    &vdsim::SimOutput::slip_angle)
        .def_readonly("steer_applied",     &vdsim::SimOutput::steer_applied)
        .def_readonly("throttle_applied",  &vdsim::SimOutput::throttle_applied)
        .def_readonly("brake_applied",     &vdsim::SimOutput::brake_applied)
        .def_readonly("rack_torque",   &vdsim::SimOutput::rack_torque)
        .def_readonly("sensors",       &vdsim::SimOutput::sensors);

    py::class_<vdsim::SimSession>(m, "SimSession")
        .def("reset",          &vdsim::SimSession::reset)
        .def("set_input",
             static_cast<void (vdsim::SimSession::*)(const vdsim::CmdL4&)>(
                 &vdsim::SimSession::set_input))
        .def("set_input",
             static_cast<void (vdsim::SimSession::*)(const vdsim::ControlInput&)>(
                 &vdsim::SimSession::set_input))
        .def("tick",           &vdsim::SimSession::tick)
        .def("state",          &vdsim::SimSession::state)
        .def("measured_state", &vdsim::SimSession::measured_state)
        .def("output",         &vdsim::SimSession::output)
        .def("sim_time",       &vdsim::SimSession::sim_time)
        .def("dynamics",       &vdsim::SimSession::dynamics,
             py::return_value_policy::reference_internal,
             "Access the plant to install subsystem modules (e.g. set_brake_module).");

    // Factory: build a SimSession (flat ground) from level + params.
    m.def("make_sim_session",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             const std::string& level, double sensor_delay_s, double mu,
             double nominal_dt, const vdsim::ActuatorParams& actuator,
             const vdsim::SolverParams& solver, const vdsim::SensorParams& sensors,
             double mu_right, double mu_boundary_y, double grade, double bank,
             double rough_amp, double rough_wavelength, int iso_class) {
              std::unique_ptr<vdsim::IVehicleDynamics> dyn =
                  (level == "K" || level == "L0") ? vdsim::create_kinematic()
                  : (level == "L1") ? vdsim::create_bicycle()
                  : (level == "L3") ? vdsim::create_fourteen_dof()
                  : (level == "L4") ? vdsim::create_fourteen_dof_kinematic()
                                    : vdsim::create_seven_dof();
              vdsim::SimConfig cfg;
              cfg.actuator       = actuator;
              cfg.sensors        = sensors;
              cfg.sensor_delay_s = sensor_delay_s;
              cfg.nominal_dt     = nominal_dt;
              // iso_class>=0 -> ISO 8608; rough_amp>0 -> two-tone rough;
              // grade/bank -> inclined; mu_right>=0 -> split; else flat.
              std::unique_ptr<vdsim::IContactProvider> ground;
              if (iso_class >= 0)
                  ground = vdsim::create_iso8608_ground(0.0, mu, iso_class, 1u);
              else if (rough_amp > 0.0)
                  ground = vdsim::create_rough_ground(0.0, mu, rough_amp, rough_wavelength);
              else if (grade != 0.0 || bank != 0.0)
                  ground = vdsim::create_inclined_ground(0.0, grade, bank, mu);
              else if (mu_right >= 0.0)
                  ground = vdsim::create_split_mu_ground(0.0, mu, mu_right, mu_boundary_y);
              else
                  ground = vdsim::create_flat_ground(0.0, mu);
              return std::make_unique<vdsim::SimSession>(
                  std::move(dyn), std::move(ground), vp, tp, solver, cfg);
          },
          py::arg("vehicle"), py::arg("tire"), py::arg("level") = "L2",
          py::arg("sensor_delay_s") = 0.0, py::arg("mu") = 1.0,
          py::arg("nominal_dt") = 0.005,
          py::arg("actuator") = vdsim::ActuatorParams{},
          py::arg("solver") = vdsim::SolverParams{},
          py::arg("sensors") = vdsim::SensorParams{},
          py::arg("mu_right") = -1.0, py::arg("mu_boundary_y") = 0.0,
          py::arg("grade") = 0.0, py::arg("bank") = 0.0,
          py::arg("rough_amp") = 0.0, py::arg("rough_wavelength") = 4.0,
          py::arg("iso_class") = -1);

    // Build a SimSession on a heightmap terrain (2D array h[ny][nx]). Per-wheel
    // bilinear height + gradient normal -> slope-gravity works on arbitrary
    // terrain (Blender mesh baked to a heightmap).
    m.def("make_sim_session_heightmap",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             const std::string& level,
             py::array_t<double, py::array::c_style | py::array::forcecast> arr,
             double x0, double y0, double dx, double dy, double mu,
             double nominal_dt, const vdsim::SolverParams& solver) {
              auto info = arr.request();
              if (info.ndim != 2)
                  throw std::runtime_error("heightmap must be a 2D array [ny][nx]");
              const int ny = (int)info.shape[0], nx = (int)info.shape[1];
              const double* p = static_cast<double*>(info.ptr);
              std::vector<double> h(p, p + (size_t)nx * ny);
              std::unique_ptr<vdsim::IVehicleDynamics> dyn =
                  (level == "K" || level == "L0") ? vdsim::create_kinematic()
                  : (level == "L1") ? vdsim::create_bicycle()
                  : (level == "L3") ? vdsim::create_fourteen_dof()
                  : (level == "L4") ? vdsim::create_fourteen_dof_kinematic()
                                    : vdsim::create_seven_dof();
              vdsim::SimConfig cfg; cfg.nominal_dt = nominal_dt;
              return std::make_unique<vdsim::SimSession>(
                  std::move(dyn),
                  vdsim::create_heightmap_ground(std::move(h), nx, ny, x0, y0, dx, dy, mu),
                  vp, tp, solver, cfg);
          },
          py::arg("vehicle"), py::arg("tire"), py::arg("level"), py::arg("heightmap"),
          py::arg("x0") = 0.0, py::arg("y0") = 0.0, py::arg("dx") = 1.0,
          py::arg("dy") = 1.0, py::arg("mu") = 1.0, py::arg("nominal_dt") = 0.005,
          py::arg("solver") = vdsim::SolverParams{});

    // Session on a PSD road. Analytic Gd0/waviness(/dual-slope), or a measured
    // (n, Gd) table when both n and gd are non-empty.
    m.def("make_sim_session_psd",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             const std::string& level, double gd_n0, double waviness,
             double n_break, double waviness_high, double n_min, double n_max,
             double mu, std::vector<double> n, std::vector<double> gd,
             double nominal_dt, const vdsim::SolverParams& solver, unsigned seed) {
              std::unique_ptr<vdsim::IVehicleDynamics> dyn =
                  (level == "K" || level == "L0") ? vdsim::create_kinematic()
                  : (level == "L1") ? vdsim::create_bicycle()
                  : (level == "L3") ? vdsim::create_fourteen_dof()
                  : (level == "L4") ? vdsim::create_fourteen_dof_kinematic()
                                    : vdsim::create_seven_dof();
              std::unique_ptr<vdsim::IContactProvider> ground =
                  (!n.empty() && n.size() == gd.size())
                  ? vdsim::create_psd_ground_table(0.0, mu, std::move(n), std::move(gd),
                                                   n_min, n_max, seed)
                  : vdsim::create_psd_ground(0.0, mu, gd_n0, waviness, n_break,
                                             waviness_high, n_min, n_max, seed);
              vdsim::SimConfig cfg; cfg.nominal_dt = nominal_dt;
              return std::make_unique<vdsim::SimSession>(
                  std::move(dyn), std::move(ground), vp, tp, solver, cfg);
          },
          py::arg("vehicle"), py::arg("tire"), py::arg("level") = "L3",
          py::arg("gd_n0") = 256e-6, py::arg("waviness") = 2.0,
          py::arg("n_break") = 0.0, py::arg("waviness_high") = 2.0,
          py::arg("n_min") = 0.011, py::arg("n_max") = 4.0, py::arg("mu") = 1.0,
          py::arg("n") = std::vector<double>{}, py::arg("gd") = std::vector<double>{},
          py::arg("nominal_dt") = 0.005, py::arg("solver") = vdsim::SolverParams{},
          py::arg("seed") = 1u);

    m.def("make_vla_plant_session",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             double z, double base_mu,
             std::vector<std::tuple<double, double, double>> patches,
             const vdsim::SolverParams& solver, double nominal_dt) {
              vdsim::SimConfig cfg;
              cfg.nominal_dt = nominal_dt;
              cfg.direct_control_path = true;
              return std::make_unique<vdsim::SimSession>(
                  vdsim::create_seven_dof(),
                  vdsim::create_friction_patch_ground(z, base_mu, patches),
                  vp, tp, solver, cfg);
          },
          py::arg("vehicle"), py::arg("tire"),
          py::arg("z") = 0.0, py::arg("base_mu") = 1.0,
          py::arg("patches") = std::vector<std::tuple<double, double, double>>{},
          py::arg("solver") = vdsim::SolverParams{},
          py::arg("nominal_dt") = 0.001);
}
