// Python bindings for vdsim_core (subset).
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "vdsim/contact.hpp"
#include "vdsim/control.hpp"
#include "vdsim/control_converter.hpp"
#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"
#include "vdsim/params.hpp"
#include "vdsim/sim_session.hpp"
#include "vdsim/scenario.hpp"
#include "vdsim/state.hpp"
#include "vdsim/suspension.hpp"

namespace py = pybind11;

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
        .def_readwrite("roll_stiffness_front", &vdsim::VehicleParams::roll_stiffness_front)
        .def_readwrite("roll_stiffness_rear",  &vdsim::VehicleParams::roll_stiffness_rear)
        .def_readwrite("spring_stiffness",     &vdsim::VehicleParams::spring_stiffness)
        .def_readwrite("damper_coefficient",   &vdsim::VehicleParams::damper_coefficient)
        .def_readwrite("unsprung_mass",        &vdsim::VehicleParams::unsprung_mass)
        .def_readwrite("drive_type",           &vdsim::VehicleParams::drive_type)
        .def_readwrite("differential",         &vdsim::VehicleParams::differential)
        .def_readwrite("max_motor_torque",     &vdsim::VehicleParams::max_motor_torque)
        .def_readwrite("max_brake_torque",     &vdsim::VehicleParams::max_brake_torque)
        .def_readwrite("brake_bias_front",     &vdsim::VehicleParams::brake_bias_front)
        .def_readwrite("brake_ebd_enabled",    &vdsim::VehicleParams::brake_ebd_enabled)
        .def_readwrite("steering_ratio",       &vdsim::VehicleParams::steering_ratio)
        .def_readwrite("max_steer_angle_wheel",&vdsim::VehicleParams::max_steer_angle_wheel)
        .def_readwrite("ackerman_percent",     &vdsim::VehicleParams::ackerman_percent)
        .def_readwrite("aero_drag_coeff",      &vdsim::VehicleParams::aero_drag_coeff)
        .def_readwrite("frontal_area",         &vdsim::VehicleParams::frontal_area)
        .def_readwrite("aero_lift_front",      &vdsim::VehicleParams::aero_lift_front)
        .def_readwrite("aero_lift_rear",       &vdsim::VehicleParams::aero_lift_rear)
        .def_readwrite("anti_dive_front",      &vdsim::VehicleParams::anti_dive_front)
        .def_readwrite("anti_squat_rear",      &vdsim::VehicleParams::anti_squat_rear)
        .def_static("from_yaml", &vdsim::VehicleParams::from_yaml)
        .def("to_yaml",          &vdsim::VehicleParams::to_yaml);

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
        .def("yaw",        &vdsim::State::yaw)
        .def("yaw_rate",   &vdsim::State::yaw_rate)
        .def("vx",         &vdsim::State::vx)
        .def("vy",         &vdsim::State::vy)
        .def("beta",       &vdsim::State::beta);

    // -------- ContactPoint --------
    py::class_<vdsim::ContactPoint>(m, "ContactPoint")
        .def(py::init<>())
        .def_readwrite("position",  &vdsim::ContactPoint::position)
        .def_readwrite("normal",    &vdsim::ContactPoint::normal)
        .def_readwrite("is_valid",  &vdsim::ContactPoint::is_valid)
        .def_readwrite("mu_long",   &vdsim::ContactPoint::mu_long)
        .def_readwrite("mu_lat",    &vdsim::ContactPoint::mu_lat)
        .def_readwrite("surface_id", &vdsim::ContactPoint::surface_id);

    // -------- CmdL4 (used by step()) --------
    py::class_<vdsim::CmdL4>(m, "CmdL4")
        .def(py::init<>())
        .def_readwrite("throttle",          &vdsim::CmdL4::throttle)
        .def_readwrite("brake",             &vdsim::CmdL4::brake)
        .def_readwrite("steer_angle_wheel", &vdsim::CmdL4::steer_angle_wheel)
        .def_readwrite("gear",              &vdsim::CmdL4::gear);

    // -------- IVehicleDynamics --------
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
        .def("wheel_slip_ratio", &vdsim::IVehicleDynamics::wheel_slip_ratio)
        .def("wheel_slip_angle", &vdsim::IVehicleDynamics::wheel_slip_angle)
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
    m.def("create_flat_ground",  &vdsim::create_flat_ground,
          py::arg("z") = 0.0, py::arg("mu") = 1.0);

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
        .def_readonly("steer_applied", &vdsim::SimOutput::steer_applied)
        .def_readonly("rack_torque",   &vdsim::SimOutput::rack_torque);

    py::class_<vdsim::SimSession>(m, "SimSession")
        .def("reset",          &vdsim::SimSession::reset)
        .def("set_input",      &vdsim::SimSession::set_input)
        .def("tick",           &vdsim::SimSession::tick)
        .def("state",          &vdsim::SimSession::state)
        .def("measured_state", &vdsim::SimSession::measured_state)
        .def("output",         &vdsim::SimSession::output)
        .def("sim_time",       &vdsim::SimSession::sim_time);

    // Factory: build a SimSession (flat ground) from level + params.
    m.def("make_sim_session",
          [](const vdsim::VehicleParams& vp, const vdsim::TireParams& tp,
             const std::string& level, double sensor_delay_s, double mu,
             double nominal_dt) {
              std::unique_ptr<vdsim::IVehicleDynamics> dyn =
                  (level == "K" || level == "L0") ? vdsim::create_kinematic()
                  : (level == "L1") ? vdsim::create_bicycle()
                  : (level == "L3") ? vdsim::create_fourteen_dof()
                                    : vdsim::create_seven_dof();
              vdsim::SimConfig cfg;
              cfg.sensor_delay_s = sensor_delay_s;
              cfg.nominal_dt     = nominal_dt;
              vdsim::SolverParams sp;
              return std::make_unique<vdsim::SimSession>(
                  std::move(dyn), vdsim::create_flat_ground(0.0, mu),
                  vp, tp, sp, cfg);
          },
          py::arg("vehicle"), py::arg("tire"), py::arg("level") = "L2",
          py::arg("sensor_delay_s") = 0.0, py::arg("mu") = 1.0,
          py::arg("nominal_dt") = 0.005);
}
