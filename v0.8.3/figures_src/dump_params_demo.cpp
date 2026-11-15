// One-shot generator for Task 12 figures and sample YAMLs.
//
// Writes:
//   docs/tasks/12_W4_params_yaml/sample_default_vehicle.yaml
//   docs/tasks/12_W4_params_yaml/sample_default_tire.yaml
//   docs/tasks/12_W4_params_yaml/sample_sports_vehicle.yaml
//   stdout: per-field absolute roundtrip residual (CSV) for both Vehicle and Tire.
//
// Build (from build dir):
//   g++ -std=c++17 -I ../core/include -I _deps/eigen-src \
//       -I _deps/yaml-cpp-src/include -I _deps/spdlog-src/include \
//       ../docs/figures_src/dump_params_demo.cpp \
//       core/CMakeFiles/vdsim_core.dir/src/params.cpp.o \
//       _deps/yaml-cpp-build/libyaml-cpp.a -lpthread -o /tmp/dump_params_demo

#include "vdsim/params.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>

namespace fs = std::filesystem;

namespace {

void print_row(const char* group, const char* field, double a, double b) {
    std::printf("%s,%s,%.6f,%.6f,%.6e\n", group, field, a, b, std::abs(a - b));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <out_dir>\n", argv[0]);
        return 2;
    }
    const fs::path out = argv[1];
    fs::create_directories(out);

    // --- 1. Default sedan ---
    const vdsim::VehicleParams v0;
    const vdsim::TireParams    t0;
    const auto v_path = out / "sample_default_vehicle.yaml";
    const auto t_path = out / "sample_default_tire.yaml";
    v0.to_yaml(v_path.string());

    // Tire has no to_yaml — emit a hand-written canonical YAML for documentation.
    {
        std::FILE* f = std::fopen(t_path.string().c_str(), "w");
        std::fprintf(f,
            "# Default tire (Pacejka MF96 simple form).\n"
            "B_long: %g\nC_long: %g\nD_long: %g\nE_long: %g\n"
            "B_lat:  %g\nC_lat:  %g\nD_lat:  %g\nE_lat:  %g\n"
            "mu_nominal: %g\nFz_nominal: %g\n"
            "cornering_stiffness: %g\nrolling_resistance: %g\n",
            t0.B_long, t0.C_long, t0.D_long, t0.E_long,
            t0.B_lat,  t0.C_lat,  t0.D_lat,  t0.E_lat,
            t0.mu_nominal, t0.Fz_nominal,
            t0.cornering_stiffness, t0.rolling_resistance);
        std::fclose(f);
    }

    // --- 2. "Sports" variant: a few tuned fields ---
    vdsim::VehicleParams vs = v0;
    vs.mass               = 1320.0;
    vs.mass_sprung        = 1180.0;
    vs.inertia_diag       = vdsim::Vec3(420.0, 1700.0, 2050.0);
    vs.wheelbase          = 2.55;
    vs.cg_to_front        = 1.20;
    vs.cg_to_rear         = 1.35;
    vs.cg_height          = 0.42;
    vs.spring_stiffness   = {{45000, 45000, 42000, 42000}};
    vs.damper_coefficient = {{4200,  4200,  3900,  3900}};
    vs.roll_stiffness_front = 48000.0;
    vs.roll_stiffness_rear  = 38000.0;
    vs.drive_type         = vdsim::VehicleParams::Drive::RWD;
    vs.max_motor_torque   = 480.0;
    vs.max_brake_torque   = 3000.0;
    vs.steering_ratio     = 12.0;
    vs.max_steer_angle_wheel = 0.55;
    vs.aero_drag_coeff    = 0.34;
    vs.frontal_area       = 2.05;
    vs.to_yaml((out / "sample_sports_vehicle.yaml").string());

    // --- 3. Roundtrip diff CSV ---
    std::printf("group,field,saved,loaded,abs_diff\n");
    const auto v_back = vdsim::VehicleParams::from_yaml(v_path.string());
    print_row("mass",     "mass",         v0.mass,        v_back.mass);
    print_row("mass",     "mass_sprung",  v0.mass_sprung, v_back.mass_sprung);
    print_row("mass",     "Ixx",          v0.inertia_diag.x(), v_back.inertia_diag.x());
    print_row("mass",     "Iyy",          v0.inertia_diag.y(), v_back.inertia_diag.y());
    print_row("mass",     "Izz",          v0.inertia_diag.z(), v_back.inertia_diag.z());
    print_row("geom",     "wheelbase",    v0.wheelbase,   v_back.wheelbase);
    print_row("geom",     "cg_to_front",  v0.cg_to_front, v_back.cg_to_front);
    print_row("geom",     "cg_to_rear",   v0.cg_to_rear,  v_back.cg_to_rear);
    print_row("geom",     "track_front",  v0.track_front, v_back.track_front);
    print_row("geom",     "track_rear",   v0.track_rear,  v_back.track_rear);
    print_row("geom",     "cg_height",    v0.cg_height,   v_back.cg_height);
    print_row("geom",     "wheel_radius", v0.wheel_radius_nominal, v_back.wheel_radius_nominal);
    for (int i = 0; i < vdsim::NUM_WHEELS; ++i) {
        const char* lbl = (i==0?"FL":i==1?"FR":i==2?"RL":"RR");
        char buf[64];
        std::snprintf(buf, sizeof(buf), "k_spring_%s", lbl);
        print_row("susp", buf, v0.spring_stiffness[i], v_back.spring_stiffness[i]);
        std::snprintf(buf, sizeof(buf), "c_damp_%s", lbl);
        print_row("susp", buf, v0.damper_coefficient[i], v_back.damper_coefficient[i]);
        std::snprintf(buf, sizeof(buf), "m_unsprung_%s", lbl);
        print_row("susp", buf, v0.unsprung_mass[i], v_back.unsprung_mass[i]);
    }
    print_row("susp",     "roll_stiff_f", v0.roll_stiffness_front, v_back.roll_stiffness_front);
    print_row("susp",     "roll_stiff_r", v0.roll_stiffness_rear,  v_back.roll_stiffness_rear);
    print_row("drive",    "drive_enum",   (double)v0.drive_type, (double)v_back.drive_type);
    print_row("drive",    "Tmot_max",     v0.max_motor_torque, v_back.max_motor_torque);
    print_row("drive",    "Tbrk_max",     v0.max_brake_torque, v_back.max_brake_torque);
    print_row("steer",    "ratio",        v0.steering_ratio, v_back.steering_ratio);
    print_row("steer",    "delta_max",    v0.max_steer_angle_wheel, v_back.max_steer_angle_wheel);
    print_row("aero",     "Cd",           v0.aero_drag_coeff, v_back.aero_drag_coeff);
    print_row("aero",     "frontal_area", v0.frontal_area, v_back.frontal_area);

    // Tire roundtrip
    const auto t_back = vdsim::TireParams::from_yaml(t_path.string());
    print_row("tire_lon", "B_long",   t0.B_long, t_back.B_long);
    print_row("tire_lon", "C_long",   t0.C_long, t_back.C_long);
    print_row("tire_lon", "D_long",   t0.D_long, t_back.D_long);
    print_row("tire_lon", "E_long",   t0.E_long, t_back.E_long);
    print_row("tire_lat", "B_lat",    t0.B_lat,  t_back.B_lat);
    print_row("tire_lat", "C_lat",    t0.C_lat,  t_back.C_lat);
    print_row("tire_lat", "D_lat",    t0.D_lat,  t_back.D_lat);
    print_row("tire_lat", "E_lat",    t0.E_lat,  t_back.E_lat);
    print_row("tire_fr",  "mu_nom",   t0.mu_nominal, t_back.mu_nominal);
    print_row("tire_fr",  "Fz_nom",   t0.Fz_nominal, t_back.Fz_nominal);
    print_row("tire_lin", "Calpha",   t0.cornering_stiffness, t_back.cornering_stiffness);
    print_row("tire_lin", "rr",       t0.rolling_resistance,  t_back.rolling_resistance);

    return 0;
}
