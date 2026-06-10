// Chrono Pac02 reference generator (standalone — links Chrono, NOT VDSim).
//
// Builds the parity reference CSV by driving Project Chrono's Pac02 tire through a
// (Fz, kappa, alpha) grid with ChTireTestRig, using the SAME coefficients as VDSim's
// evaluator: it parses the shared sample_pac02.tir and emits the equivalent Chrono
// Pac02 JSON (coeffs are inline in Chrono's JSON), so the input is identical on both
// sides (no transcription). Output forces are rotated into the wheel/ISO frame to
// match VDSim's convention and written to reference/pac02_reference.csv.
//
// This file is compiled by external/chrono_parity/CMakeLists.txt against an existing
// Chrono build — it is never part of VDSim's own CMake/core.

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

#include <iomanip>

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;

// ---- minimal wheel (mass/inertia/size only; the rig needs a ChWheel) ----
class GenWheel : public ChWheel {
  public:
    GenWheel() : ChWheel("gen"), m_inertia(0.1, 0.2, 0.1) {}
    double GetWheelMass() const override { return 10.0; }
    const ChVector<>& GetWheelInertia() const override { return m_inertia; }
    double GetRadius() const override { return 0.31; }
    double GetWidth() const override { return 0.205; }
  private:
    ChVector<> m_inertia;
};

// ---- parse the flat TNO .tir (key=value, uppercase keys) ----
std::map<std::string, double> parse_tir(const std::string& path) {
    std::map<std::string, double> c;
    std::ifstream in(path);
    if (!in) { std::cerr << "cannot open " << path << "\n"; std::exit(2); }
    std::string line;
    while (std::getline(in, line)) {
        auto h = line.find('$'); if (h != std::string::npos) line = line.substr(0, h);
        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string k = line.substr(0, eq), v = line.substr(eq + 1);
        auto trim = [](std::string& s) {
            const char* ws = " \t\r\n'";
            auto a = s.find_first_not_of(ws); auto b = s.find_last_not_of(ws);
            s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
        };
        trim(k); trim(v);
        if (k.empty() || v.empty()) continue;
        for (auto& ch : k) ch = std::toupper(ch);
        try { c[k] = std::stod(v); } catch (...) {}
    }
    return c;
}

// ---- emit the Chrono Pac02 JSON with coeffs inline (lowercase keys) ----
std::string to_chrono_json(const std::map<std::string, double>& c) {
    auto g = [&](const char* up, double dflt) {
        auto it = c.find(up); return it == c.end() ? dflt : it->second;
    };
    std::ostringstream o;
    o << std::setprecision(12);
    auto kv = [&](const char* jkey, const char* tkey, double dflt, bool comma = true) {
        o << "    \"" << jkey << "\": " << g(tkey, dflt) << (comma ? ",\n" : "\n");
    };
    o << "{\n";
    o << "  \"Name\": \"sample_pac02\",\n  \"Type\": \"Tire\",\n  \"Template\": \"Pac02Tire\",\n";
    o << "  \"Mass\": 11.5,\n  \"Inertia\": [0.156, 0.679, 0.156],\n";
    o << "  \"Use Mode\": 3,\n  \"Coefficient of Friction\": 1.0,\n  \"Tire Side\": \"left\",\n";
    o << "  \"Dimension\": {\n";
    o << "    \"Unloaded Radius\": " << g("UNLOADED_RADIUS", 0.31) << ",\n";
    o << "    \"Width\": " << g("WIDTH", 0.205) << ",\n    \"Aspect Ratio\": " << g("ASPECT_RATIO", 0.55) << ",\n";
    o << "    \"Rim Radius\": " << g("RIM_RADIUS", 0.1905) << ",\n    \"Rim Width\": " << g("RIM_WIDTH", 0.16) << "\n  },\n";
    o << "  \"Vertical\": {\n";
    o << "    \"Vertical Stiffness\": " << g("VERTICAL_STIFFNESS", 220000) << ",\n";
    o << "    \"Vertical Damping\": " << g("VERTICAL_DAMPING", 300) << ",\n";
    o << "    \"Nominal Wheel Load\": " << g("FNOMIN", 4000) << "\n  },\n";

    // Scaling — all 1.0 unless present.
    o << "  \"Scaling Factors\": {\n";
    const char* sc[] = {"lfz0","lcx","lmux","lex","lkx","lhx","lvx","lgax","lcy","lmuy","ley",
        "lky","lhy","lvy","lgay","ltr","lres","lgaz","lxal","lyka","lvyka","ls","lsgkp","lsgal",
        "lgyr","lmx","lvmx","lmy"};
    for (size_t i = 0; i < sizeof(sc)/sizeof(sc[0]); ++i) {
        std::string up = sc[i]; for (auto& ch : up) ch = std::toupper(ch);
        o << "    \"" << sc[i] << "\": " << g(up.c_str(), 1.0)
          << (i + 1 < sizeof(sc)/sizeof(sc[0]) ? ",\n" : "\n");
    }
    o << "  },\n";

    o << "  \"Longitudinal Coefficients\": {\n";
    kv("pcx1","PCX1",1.65); kv("pdx1","PDX1",1.0); kv("pdx2","PDX2",0); kv("pdx3","PDX3",0);
    kv("pex1","PEX1",0); kv("pex2","PEX2",0); kv("pex3","PEX3",0); kv("pex4","PEX4",0);
    kv("pkx1","PKX1",20); kv("pkx2","PKX2",0); kv("pkx3","PKX3",0);
    kv("phx1","PHX1",0); kv("phx2","PHX2",0); kv("pvx1","PVX1",0); kv("pvx2","PVX2",0);
    kv("rbx1","RBX1",13); kv("rbx2","RBX2",0); kv("rcx1","RCX1",1);
    kv("rex1","REX1",0); kv("rex2","REX2",0); kv("rhx1","RHX1",0,false);
    o << "  },\n";

    o << "  \"Lateral Coefficients\": {\n";
    kv("pcy1","PCY1",1.3); kv("pdy1","PDY1",1.0); kv("pdy2","PDY2",0); kv("pdy3","PDY3",0);
    kv("pey1","PEY1",0); kv("pey2","PEY2",0); kv("pey3","PEY3",0); kv("pey4","PEY4",0);
    kv("pky1","PKY1",-15); kv("pky2","PKY2",2); kv("pky3","PKY3",0);
    kv("phy1","PHY1",0); kv("phy2","PHY2",0); kv("phy3","PHY3",0);
    kv("pvy1","PVY1",0); kv("pvy2","PVY2",0); kv("pvy3","PVY3",0); kv("pvy4","PVY4",0);
    kv("rby1","RBY1",10); kv("rby2","RBY2",0); kv("rby3","RBY3",0); kv("rcy1","RCY1",1);
    kv("rey1","REY1",0); kv("rey2","REY2",0); kv("rhy1","RHY1",0); kv("rhy2","RHY2",0);
    kv("rvy1","RVY1",0); kv("rvy2","RVY2",0); kv("rvy3","RVY3",0); kv("rvy4","RVY4",0);
    kv("rvy5","RVY5",0); kv("rvy6","RVY6",0,false);
    o << "  },\n";

    o << "  \"Aligning Coefficients\": {\n";
    kv("qbz1","QBZ1",6); kv("qbz2","QBZ2",0); kv("qbz3","QBZ3",0); kv("qbz4","QBZ4",0);
    kv("qbz5","QBZ5",0); kv("qbz9","QBZ9",0); kv("qbz10","QBZ10",0);
    kv("qcz1","QCZ1",1.05); kv("qdz1","QDZ1",0.1); kv("qdz2","QDZ2",0); kv("qdz3","QDZ3",0);
    kv("qdz4","QDZ4",0); kv("qdz6","QDZ6",0); kv("qdz7","QDZ7",0); kv("qdz8","QDZ8",0);
    kv("qdz9","QDZ9",0); kv("qez1","QEZ1",0); kv("qez2","QEZ2",0); kv("qez3","QEZ3",0);
    kv("qez4","QEZ4",0); kv("qez5","QEZ5",0);
    kv("qhz1","QHZ1",0); kv("qhz2","QHZ2",0); kv("qhz3","QHZ3",0); kv("qhz4","QHZ4",0);
    kv("ssz1","SSZ1",0); kv("ssz2","SSZ2",0); kv("ssz3","SSZ3",0); kv("ssz4","SSZ4",0,false);
    o << "  },\n";

    o << "  \"Overturning Coefficients\": {\n";
    kv("qsx1","QSX1",0); kv("qsx2","QSX2",0); kv("qsx3","QSX3",0,false);
    o << "  },\n";

    o << "  \"Rolling Coefficients\": {\n";
    kv("qsy1","QSY1",0.01); kv("qsy2","QSY2",0); kv("qsy3","QSY3",0); kv("qsy4","QSY4",0,false);
    o << "  }\n}\n";
    return o.str();
}

int main(int argc, char** argv) {
    const std::string here = (argc > 1) ? argv[1] : ".";
    const std::string tir = here + "/sample_pac02.tir";
    const std::string out_csv = here + "/reference/pac02_reference.csv";

    const auto coeff = parse_tir(tir);
    const std::string json = to_chrono_json(coeff);
    const std::string tmp_json = here + "/_pac02_chrono.json";
    { std::ofstream(tmp_json) << json; }

    const std::vector<double> FZ = {2000, 4000, 6000};
    const std::vector<double> KAPPA = {-0.15, -0.08, -0.03, 0.0, 0.03, 0.08, 0.15};
    std::vector<double> ALPHA;
    for (double a : {-8.0, -4.0, -1.0, 0.0, 1.0, 4.0, 8.0}) ALPHA.push_back(a * CH_C_DEG_TO_RAD);
    const double VX = 16.0;

    std::ofstream csv(out_csv);
    csv << "Fz,kappa,alpha,gamma,Fx,Fy,Mz\n";
    csv << std::setprecision(10);

    int n = 0;
    for (double Fz : FZ) {
        for (double kappa : KAPPA) {
            for (double alpha : ALPHA) {
                // Fresh system + rig per point (no re-init ambiguity).
                ChSystemSMC sys;
                sys.Set_G_acc(ChVector<>(0, 0, -9.81));
                auto tire = ReadTireJSON(tmp_json);
                auto wheel = chrono_types::make_shared<GenWheel>();

                ChTireTestRig rig(wheel, tire, &sys);
                rig.SetGravitationalAcceleration(9.81);
                rig.SetNormalLoad(Fz);
                rig.SetCamberAngle(0.0);
                rig.SetTireStepsize(1e-4);
                // mu=1 flat rigid; long patch so the carrier (moving at VX) does not
                // run off the terrain before we sample (default 10 m runs out at 0.6 s).
                rig.SetTerrainRigid(1.0, 0.0, 2e7, 200.0);
                rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunction_Const>(alpha));
                rig.Initialize(kappa, VX);   // sets long/ang speed for target slip

                // Run to steady, then average the in-contact force over the tail to
                // reject any residual vertical-mode ripple (airborne steps have Fz=0).
                // Run to steady, then average over the in-contact tail. Record the
                // ACTUAL slip Chrono used (GetLongitudinalSlip/GetSlipAngle), not the
                // commanded value: the rig's commanded long slip differs from Pac02's
                // internal slip (effective rolling radius), so comparing both models at
                // Chrono's actual slip removes a spurious longitudinal offset.
                const double ca = std::cos(alpha), sa = std::sin(alpha);
                double sFx = 0, sFy = 0, sFz = 0, sMz = 0, sK = 0, sA = 0; int nc = 0;
                for (int i = 0; i < 5000; ++i) {
                    rig.Advance(1e-4);
                    if (i >= 3500) {
                        const TerrainForce t = rig.ReportTireForce();
                        if (t.force.z() > 1.0) {  // in contact
                            // Global -> wheel frame. The rig yaws the wheel by the slip
                            // angle about +Z (heading=(cos a, sin a), lateral=(-sin a,
                            // cos a)), so invert to recover the tire-frame components.
                            sFx += ca * t.force.x() - sa * t.force.y();
                            sFy += sa * t.force.x() + ca * t.force.y();
                            sFz += t.force.z();
                            sMz += t.moment.z();
                            sK  += tire->GetLongitudinalSlip();
                            sA  += tire->GetSlipAngle();
                            ++nc;
                        }
                    }
                }
                if (!nc) continue;
                // Record ACTUAL slip + Fz: the rig's commanded slip differs from Pac02's
                // internal slip (effective rolling radius) and the contact Fz ripples, so
                // both models must be evaluated at the same point Chrono actually used.
                const double Fx = sFx / nc, Fy = sFy / nc, Mz = sMz / nc;
                const double Fz_act = sFz / nc, k_act = sK / nc, a_act = sA / nc;

                csv << Fz_act << "," << k_act << "," << a_act << ",0,"
                    << Fx << "," << Fy << "," << Mz << "\n";
                ++n;
            }
        }
    }
    std::cout << "[gen] wrote " << n << " rows -> " << out_csv << "\n";
    return 0;
}
