// Chrono KC reference generator (standalone — links Chrono::Vehicle, NOT VDSim).
//
// Builds a Chrono::Vehicle DoubleWishbone with the SAME hardpoints as VDSim's
// configs/parts/susp_kinematics/kin/dw_front_sports.yaml, drives a ChSuspensionTestRig
// through a wheel-travel sweep, and writes camber / toe / track / caster vs travel to
// reference/kc_dw_front_reference.csv.
//
// CSV format (header then rows): travel_m,steer_m,camber_deg,toe_deg,track_mm,caster_deg

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

const double kRadToDeg = CH_C_RAD_TO_DEG;
constexpr double kPostLimit = 0.60;
constexpr int kTravelPoints = 21;
constexpr double kTravelMin = -0.06;
constexpr double kTravelMax = 0.06;

struct Hardpoints {
    ChVector<> wheel_center, uca_cf, uca_cr, uca_k, lca_cf, lca_cr, lca_k;
    ChVector<> tr_rack, tr_knuckle, sd_chassis, sd_lca;
};

std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    const auto a = s.find_first_not_of(ws);
    if (a == std::string::npos) return "";
    const auto b = s.find_last_not_of(ws);
    return s.substr(a, b - a + 1);
}


bool load_hardpoints(const std::string& yaml_path, Hardpoints& hp) {
    std::ifstream in(yaml_path);
    if (!in) return false;
    std::string section, key;
    double v[3] {};
    int n = 0;

    auto assign = [&](const ChVector<>& p) {
        if (section == "wheel" && key == "center") hp.wheel_center = p;
        else if (section == "uca" && key == "chassis_front") hp.uca_cf = p;
        else if (section == "uca" && key == "chassis_rear") hp.uca_cr = p;
        else if (section == "uca" && key == "knuckle") hp.uca_k = p;
        else if (section == "lca" && key == "chassis_front") hp.lca_cf = p;
        else if (section == "lca" && key == "chassis_rear") hp.lca_cr = p;
        else if (section == "lca" && key == "knuckle") hp.lca_k = p;
        else if (section == "tie_rod" && key == "rack") hp.tr_rack = p;
        else if (section == "tie_rod" && key == "knuckle") hp.tr_knuckle = p;
        else if (section == "spring_damper" && key == "chassis") hp.sd_chassis = p;
        else if (section == "spring_damper" && key == "lca") hp.sd_lca = p;
    };

    std::string line;
    while (std::getline(in, line)) {
        const auto raw = line;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const int indent = static_cast<int>(raw.find_first_not_of(' '));
        if (line.back() == ':' && line.rfind("- ", 0) != 0) {
            const std::string name = line.substr(0, line.size() - 1);
            if (indent == 0) {
                section = name;
                key.clear();
            } else {
                key = name;
                n = 0;
            }
            continue;
        }
        if (line.rfind("- ", 0) == 0) {
            v[n++] = std::stod(line.substr(2));
            if (n == 3) {
                assign(ChVector<>(v[0], v[1], v[2]));
                n = 0;
            }
        }
    }
    return hp.wheel_center.Length() > 0.0;
}

ChVector<> mid(const ChVector<>& a, const ChVector<>& b) {
    return 0.5 * (a + b);
}

void write_json_array(std::ostream& o, const ChVector<>& v) {
    o << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
}

bool write_suspension_json(const std::string& path, const Hardpoints& hp) {
    const ChVector<> upright = mid(hp.uca_k, hp.lca_k);
    const ChVector<> uca_cm = mid(mid(hp.uca_cf, hp.uca_cr), hp.uca_k);
    const ChVector<> lca_cm = mid(mid(hp.lca_cf, hp.lca_cr), hp.lca_k);
    const double spring_len = (hp.sd_chassis - hp.sd_lca).Length();

    std::ofstream o(path);
    if (!o) return false;
    o << std::setprecision(12);
    o << "{\n";
    o << "  \"Name\": \"VDSim dw_front_sports\",\n";
    o << "  \"Type\": \"Suspension\",\n";
    o << "  \"Template\": \"DoubleWishbone\",\n";
    o << "  \"Camber Angle (deg)\": 0,\n";
    o << "  \"Toe Angle (deg)\": 0,\n";
    o << "  \"Spindle\": {\n";
    o << "    \"Mass\": 12.0,\n    \"COM\": ";
    write_json_array(o, hp.wheel_center);
    o << ",\n    \"Inertia\": [0.24, 0.42, 0.24],\n";
    o << "    \"Radius\": 0.33,\n    \"Width\": 0.205\n  },\n";
    o << "  \"Upright\": {\n";
    o << "    \"Mass\": 5.0,\n    \"COM\": ";
    write_json_array(o, upright);
    o << ",\n    \"Moments of Inertia\": [0.02, 0.02, 0.02],\n";
    o << "    \"Products of Inertia\": [0, 0, 0],\n    \"Radius\": 0.03\n  },\n";
    o << "  \"Upper Control Arm\": {\n";
    o << "    \"Mass\": 3.0,\n    \"COM\": ";
    write_json_array(o, uca_cm);
    o << ",\n    \"Moments of Inertia\": [0.01, 0.01, 0.01],\n";
    o << "    \"Products of Inertia\": [0, 0, 0],\n    \"Radius\": 0.02,\n";
    o << "    \"Location Chassis Front\": ";
    write_json_array(o, hp.uca_cf);
    o << ",\n    \"Location Chassis Back\": ";
    write_json_array(o, hp.uca_cr);
    o << ",\n    \"Location Upright\": ";
    write_json_array(o, hp.uca_k);
    o << "\n  },\n";
    o << "  \"Lower Control Arm\": {\n";
    o << "    \"Mass\": 4.0,\n    \"COM\": ";
    write_json_array(o, lca_cm);
    o << ",\n    \"Moments of Inertia\": [0.01, 0.01, 0.01],\n";
    o << "    \"Products of Inertia\": [0, 0, 0],\n    \"Radius\": 0.03,\n";
    o << "    \"Location Chassis Front\": ";
    write_json_array(o, hp.lca_cf);
    o << ",\n    \"Location Chassis Back\": ";
    write_json_array(o, hp.lca_cr);
    o << ",\n    \"Location Upright\": ";
    write_json_array(o, hp.lca_k);
    o << "\n  },\n";
    o << "  \"Tierod\": {\n";
    o << "    \"Location Chassis\": ";
    write_json_array(o, hp.tr_rack);
    o << ",\n    \"Location Upright\": ";
    write_json_array(o, hp.tr_knuckle);
    o << "\n  },\n";
    o << "  \"Spring\": {\n";
    o << "    \"Location Chassis\": ";
    write_json_array(o, hp.sd_chassis);
    o << ",\n    \"Location Arm\": ";
    write_json_array(o, hp.sd_lca);
    o << ",\n    \"Spring Coefficient\": 180000.0,\n";
    o << "    \"Free Length\": " << spring_len << "\n  },\n";
    o << "  \"Shock\": {\n";
    o << "    \"Location Chassis\": ";
    write_json_array(o, hp.sd_chassis);
    o << ",\n    \"Location Arm\": ";
    write_json_array(o, hp.sd_lca);
    o << ",\n    \"Damping Coefficient\": 6000.0\n  },\n";
    o << "  \"Axle\": { \"Inertia\": 0.4 }\n";
    o << "}\n";
    return true;
}

bool write_vehicle_json(const std::string& path, const std::string& susp_rel) {
    std::ofstream o(path);
    if (!o) return false;
    o << "{\n";
    o << "  \"Name\": \"VDSim KC reference vehicle\",\n";
    o << "  \"Type\": \"Vehicle\",\n";
    o << "  \"Template\": \"WheeledVehicle\",\n";
    o << "  \"Chassis\": { \"Input File\": \"generic/chassis/Chassis.json\" },\n";
    o << "  \"Axles\": [\n";
    o << "    {\n";
    o << "      \"Suspension Input File\": \"" << susp_rel << "\",\n";
    o << "      \"Suspension Location\": [0, 0, 0],\n";
    o << "      \"Left Wheel Input File\": \"generic/wheel/WheelSimple.json\",\n";
    o << "      \"Right Wheel Input File\": \"generic/wheel/WheelSimple.json\",\n";
    o << "      \"Left Brake Input File\": \"generic/brake/BrakeSimple.json\",\n";
    o << "      \"Right Brake Input File\": \"generic/brake/BrakeSimple.json\",\n";
    o << "      \"Tire Input File\": \"generic/tire/RigidTire.json\"\n";
    o << "    }\n";
    o << "  ],\n";
    o << "  \"Wheelbase\": 2.5\n";
    o << "}\n";
    return true;
}

struct KcSample {
    double travel_m, camber_deg, toe_deg, track_mm, caster_deg;
};

// VDSim ISO reduction (left corner, matches pose_from_spin in multibody_hard_dae.cpp).
KcSample sample_left_spindle(ChSuspensionTestRig& rig, int axle) {
    const ChQuaternion<> rot = rig.GetSpindleRot(axle, LEFT);
    const ChVector<> spin = rot.GetYaxis();
    const ChVector<> pos = rig.GetSpindlePos(axle, LEFT);
    KcSample s;
    s.travel_m = rig.GetWheelTravel(axle, LEFT);
    s.toe_deg = std::atan2(spin.x(), spin.y()) * kRadToDeg;
    s.camber_deg = std::atan2(-spin.z(), std::abs(spin.y())) * kRadToDeg;
    s.track_mm = 2.0 * pos.y() * 1000.0;
    s.caster_deg = 0.0;
    return s;
}

class SweepDriverSTR : public ChDriverSTR {
  public:
    void set_target(double norm) { target_ = norm; }

  private:
    void Synchronize(double time) override {
        ChDriverSTR::Synchronize(time);
        SetDisplacementLeft(0, target_);
        SetDisplacementRight(0, target_);
        SetSteering(0.0);
    }

    double target_ = 0.0;
};

std::string chrono_vehicle_data_dir(const std::string& chrono_dir) {
    namespace fs = std::filesystem;
    const fs::path p(chrono_dir);
    if (fs::exists(p / "data" / "vehicle")) return (p / "data" / "vehicle").string() + "/";
    if (fs::exists(p.parent_path() / "data" / "vehicle"))
        return (p.parent_path() / "data" / "vehicle").string() + "/";
    return vehicle::GetDataPath();
}

void settle_rig(ChSuspensionTestRig& rig, double dt, int steps = 2000) {
    for (int i = 0; i < steps; ++i) rig.Advance(dt);
}

double drive_to_travel(ChSuspensionTestRig& rig, SweepDriverSTR& driver, double target_travel,
                       double dt) {
    double lo = -1.0, hi = 1.0;
    for (int iter = 0; iter < 16; ++iter) {
        const double mid = 0.5 * (lo + hi);
        driver.set_target(mid);
        settle_rig(rig, dt, 400);
        const double tr = rig.GetWheelTravel(0, LEFT);
        if (tr < target_travel) lo = mid;
        else hi = mid;
    }
    driver.set_target(0.5 * (lo + hi));
    settle_rig(rig, dt, 2000);
    return rig.GetWheelTravel(0, LEFT);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string repo = (argc > 1) ? argv[1] : ".";
    const std::string chrono_root = (argc > 2) ? argv[2] : "";
    const std::string yaml_path = repo + "/configs/parts/susp_kinematics/kin/dw_front_sports.yaml";
    const std::string out_path = repo + "/external/chrono_kc/reference/kc_dw_front_reference.csv";

    Hardpoints hp;
    if (!load_hardpoints(yaml_path, hp)) {
        std::cerr << "cannot load hardpoints from " << yaml_path << "\n";
        return 2;
    }

    if (!chrono_root.empty()) {
        SetChronoDataPath(chrono_root + "/");
        SetDataPath(chrono_vehicle_data_dir(chrono_root));
    }

    namespace fs = std::filesystem;
    const std::string overlay = GetDataPath() + "vdsim_kc/";
    fs::create_directories(overlay);
    const std::string susp_json = overlay + "dw_front_sports.json";
    const std::string veh_json = overlay + "vehicle_kc.json";
    if (!write_suspension_json(susp_json, hp)) {
        std::cerr << "cannot write " << susp_json << "\n";
        return 2;
    }
    if (!write_vehicle_json(veh_json, "vdsim_kc/dw_front_sports.json")) {
        std::cerr << "cannot write " << veh_json << "\n";
        return 2;
    }

    auto vehicle = chrono_types::make_shared<WheeledVehicle>(veh_json, ChContactMethod::SMC, false, true);
    auto rig = chrono_types::make_shared<ChSuspensionTestRigPlatform>(vehicle, std::vector<int>{0}, kPostLimit);
    auto driver = chrono_types::make_shared<SweepDriverSTR>();
    rig->SetDriver(driver);
    rig->Initialize();

    std::vector<KcSample> rows;
    rows.reserve(kTravelPoints);
    const double dt = 1e-3;
    for (int i = 0; i < kTravelPoints; ++i) {
        const double travel_cmd = kTravelMin + (kTravelMax - kTravelMin) * i / (kTravelPoints - 1);
        const double achieved = drive_to_travel(*rig, *driver, travel_cmd, dt);
        KcSample s = sample_left_spindle(*rig, 0);
        s.travel_m = achieved;
        if (!rows.empty() && std::abs(rows.back().travel_m - s.travel_m) < 1e-5) continue;
        rows.push_back(s);
    }

    std::ofstream o(out_path);
    if (!o) {
        std::cerr << "cannot open " << out_path << "\n";
        return 2;
    }
    o << std::setprecision(9);
    o << "travel_m,steer_m,camber_deg,toe_deg,track_mm,caster_deg\n";
    for (const auto& r : rows) {
        o << r.travel_m << ",0," << r.camber_deg << "," << r.toe_deg << ","
          << r.track_mm << "," << r.caster_deg << "\n";
    }

    std::cerr << "[gen_kc_reference] wrote " << rows.size() << " rows to " << out_path << "\n";
    return 0;
}
