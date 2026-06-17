// Chrono KC reference generator (standalone — links Chrono::Vehicle, NOT VDSim).
//
// Builds a Chrono::Vehicle DoubleWishbone with the SAME hardpoints as VDSim's
// configs/parts/susp_kinematics/kin/dw_front_sports.yaml, drives a ChSuspensionTestRig
// through a wheel-travel (and steer) sweep, and writes the resulting camber / toe / track /
// caster vs travel to reference/kc_dw_front_reference.csv. The VDSim-side gate
// (tests/parity/test_chrono_kc_parity.cpp) then bands its hard-joint DAE travel maps against
// this CSV — an independent constrained-MBD cross-check of the L5 emergent geometry.
//
// This file is compiled by external/chrono_kc/CMakeLists.txt against an existing Chrono build;
// it is never part of VDSim's own CMake/core. SKELETON: the Chrono API specifics
// (PointId enum spelling, ChSuspensionTestRig construction/readout) vary by Chrono version —
// fill the TODOs against the installed Chrono and verify the output frame matches VDSim's
// ISO 8855 convention (toe +, camber +, track outward) before trusting the gate.
//
// CSV format (header then rows): travel_m,steer_m,camber_deg,toe_deg,track_mm,caster_deg

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

// ---- Hardpoints, in the suspension reference frame, from dw_front_sports.yaml. ----
// KEEP IN SYNC with the YAML (the shared input — same role as sample_pac02.tir for tyres).
// A tiny flat-YAML reader is preferred over transcription; inlined here as the skeleton.
// TODO: read these straight from $1/configs/parts/susp_kinematics/kin/dw_front_sports.yaml
// so the two sides can never drift. Keys: wheel.center, uca.{chassis_front,chassis_rear,
// knuckle}, lca.{...}, tie_rod.{rack,knuckle}, spring_damper.{chassis,lca}.
struct Hardpoints {
    ChVector<> wheel_center, uca_cf, uca_cr, uca_k, lca_cf, lca_cr, lca_k;
    ChVector<> tr_rack, tr_knuckle, sd_chassis, sd_lca;
};

// Map VDSim's hardpoint names -> Chrono ChDoubleWishbone::PointId. (Chrono's UCA/LCA are
// defined by two chassis points + the upright/knuckle point; spring & shock share the LCA
// attach in this layout.) TODO: confirm the exact PointId set for the Chrono version.
class VdsimDoubleWishbone : public ChDoubleWishbone {
public:
    explicit VdsimDoubleWishbone(const Hardpoints& hp) : ChDoubleWishbone("vdsim_dw"), hp_(hp) {}
    // ChVector<> getLocation(PointId which) override {
    //     switch (which) {
    //         case SPINDLE:  return hp_.wheel_center;
    //         case UPRIGHT:  return 0.5 * (hp_.uca_k + hp_.lca_k);
    //         case UCA_F:    return hp_.uca_cf;   case UCA_B: return hp_.uca_cr;
    //         case UCA_U:    return hp_.uca_k;
    //         case LCA_F:    return hp_.lca_cf;   case LCA_B: return hp_.lca_cr;
    //         case LCA_U:    return hp_.lca_k;
    //         case TIEROD_C: return hp_.tr_rack;  case TIEROD_U: return hp_.tr_knuckle;
    //         case SPRING_C: return hp_.sd_chassis; case SPRING_A: return hp_.sd_lca;
    //         case SHOCK_C:  return hp_.sd_chassis; case SHOCK_A:  return hp_.sd_lca;
    //         default:       return ChVector<>(0, 0, 0);
    //     }
    // }
    // ... plus mass/inertia/spring/damper getters required by ChDoubleWishbone (use the
    // VDSim VehicleParams: spring_stiffness, damper_coefficient, unsprung_mass).
private:
    Hardpoints hp_;
};

}  // namespace

int main(int argc, char** argv) {
    const std::string repo = (argc > 1) ? argv[1] : ".";
    const std::string out_path = repo + "/external/chrono_kc/reference/kc_dw_front_reference.csv";

    Hardpoints hp;  // TODO: load from the YAML (see note above).

    // 1) Instantiate the suspension + a ChSuspensionTestRig on it.
    //    auto susp = chrono_types::make_shared<VdsimDoubleWishbone>(hp);
    //    ChSuspensionTestRigPlatform rig({susp}, /*steerable*/ true, ...);
    //    rig.Initialize();
    //
    // 2) Sweep wheel travel (steer = 0), then steer (travel = 0). At each step read the
    //    wheel state and reduce to camber / toe / track / caster in the ISO frame.
    //    NOTE the rig prescribes travel via the post displacement; recover the actual
    //    vertical wheel-centre travel z_v to use as the abscissa (matches VDSim's z_v).

    std::ofstream o(out_path);
    if (!o) { std::cerr << "cannot open " << out_path << "\n"; return 2; }
    o << "travel_m,steer_m,camber_deg,toe_deg,track_mm,caster_deg\n";

    const int N = 21;
    for (int i = 0; i < N; ++i) {
        const double travel = -0.06 + 0.12 * i / (N - 1);
        (void)travel;
        // const auto k = read_kc_at(rig, travel, /*steer*/ 0.0);
        // o << travel << ",0," << k.camber_deg << "," << k.toe_deg << ","
        //   << k.track_mm << "," << k.caster_deg << "\n";
    }
    // (optional) steer sweep rows with travel = 0 for future bump-steer cross-check.

    std::cerr << "[gen_kc_reference] SKELETON — fill the Chrono API TODOs, then this writes "
              << out_path << "\n";
    return 0;
}
