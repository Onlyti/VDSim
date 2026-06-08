#include "vdsim/multibody.hpp"

#include <algorithm>
#include <cmath>

namespace vdsim::mb {

namespace {

void add_bushing(std::vector<Bushing>& out,
                 const std::string& id,
                 const std::string& body_a,
                 const std::string& body_b,
                 const Vec3& k_t,
                 const Vec3& k_r,
                 const Vec3& c_t = Vec3(8.0e3, 8.0e3, 4.0e3),
                 const Vec3& c_r = Vec3(40.0, 30.0, 60.0)) {
    Bushing b;
    b.id = id;
    b.body_a_id = body_a;
    b.body_b_id = body_b;
    b.k_translation = k_t;
    b.k_rotation = k_r;
    b.c_translation = c_t;
    b.c_rotation = c_r;
    out.push_back(b);
}

void add_macpherson_bushings(std::vector<Bushing>& out) {
    add_bushing(out, "lca_front_bush", "chassis", "lca",
                Vec3(1.2e6, 1.2e6, 6.0e5), Vec3(2.5e4, 1.2e4, 4.0e4));
    add_bushing(out, "lca_rear_bush", "chassis", "lca",
                Vec3(1.0e6, 1.0e6, 5.0e5), Vec3(2.0e4, 1.0e4, 3.5e4));
    add_bushing(out, "tie_inner_bush", "chassis", "tie_rod",
                Vec3(5.0e5, 5.0e5, 3.0e5), Vec3(8.0e3, 6.0e3, 1.2e4));
}

void add_dw_bushings(std::vector<Bushing>& out) {
    add_bushing(out, "lca_front_bush", "chassis", "lca",
                Vec3(1.2e6, 1.2e6, 6.0e5), Vec3(2.5e4, 1.2e4, 4.0e4));
    add_bushing(out, "lca_rear_bush", "chassis", "lca",
                Vec3(1.0e6, 1.0e6, 5.0e5), Vec3(2.0e4, 1.0e4, 3.5e4));
    add_bushing(out, "uca_front_bush", "chassis", "uca",
                Vec3(1.0e6, 1.0e6, 5.0e5), Vec3(1.8e4, 1.0e4, 2.5e4));
    add_bushing(out, "uca_rear_bush", "chassis", "uca",
                Vec3(9.0e5, 9.0e5, 4.5e5), Vec3(1.6e4, 9.0e3, 2.2e4));
    add_bushing(out, "tie_inner_bush", "chassis", "tie_rod",
                Vec3(5.0e5, 5.0e5, 3.0e5), Vec3(8.0e3, 6.0e3, 1.2e4));
}

void add_ta_bushings(std::vector<Bushing>& out) {
    add_bushing(out, "arm_bush", "chassis", "arm",
                Vec3(8.0e5, 8.0e5, 4.0e5), Vec3(1.5e4, 2.0e4, 3.0e4));
}

void add_5link_bushings(std::vector<Bushing>& out) {
    for (const char* link :
         {"upper_fore", "upper_aft", "lower_fore", "lower_aft", "toe_link"}) {
        add_bushing(out, std::string(link) + "_bush", "chassis", link,
                    Vec3(7.0e5, 7.0e5, 3.5e5), Vec3(1.2e4, 1.0e4, 2.0e4));
    }
}

double bushing_steer_weight(const Bushing& b) {
    if (b.id.find("tie") != std::string::npos) return 1.0;
    if (b.id.find("lca") != std::string::npos) return 0.35;
    if (b.id.find("toe_link") != std::string::npos) return 0.9;
    return 0.2;
}

double bushing_camber_weight(const Bushing& b) {
    if (b.id.find("uca") != std::string::npos) return 1.0;
    if (b.id.find("lca") != std::string::npos) return 0.55;
    if (b.id.find("upper") != std::string::npos) return 0.8;
    if (b.id.find("lower") != std::string::npos) return 0.5;
    return 0.15;
}

}  // namespace

void ensure_default_bushings(SuspensionTopology& topo) {
    if (!topo.bushings.empty()) return;
    switch (topo.kind) {
        case TopologyKind::MacPherson:     add_macpherson_bushings(topo.bushings); break;
        case TopologyKind::DoubleWishbone: add_dw_bushings(topo.bushings); break;
        case TopologyKind::TrailingArm:    add_ta_bushings(topo.bushings); break;
        case TopologyKind::MultiLink5:     add_5link_bushings(topo.bushings); break;
        default: break;
    }
}

void compliance_targets_rad(const SuspensionTopology& topo, const WheelLoad& load,
                            double& toe_rad, double& camber_rad) {
    toe_rad = 0.0;
    camber_rad = 0.0;
    const double Fy = load.force_world.y();
    const double Fz = load.force_world.z();
    const double Mz = load.moment_world.z();
    constexpr double kToeArm = 0.075;
    constexpr double kCamberArm = 0.045;
    for (const auto& b : topo.bushings) {
        const double w_steer = bushing_steer_weight(b);
        const double w_camber = bushing_camber_weight(b);
        const double Kz = std::max(1.0, b.k_rotation.z());
        const double Kx = std::max(1.0, b.k_rotation.x());
        toe_rad += w_steer * (Fy * kToeArm + Mz) / Kz;
        camber_rad += w_camber * (Fy * kCamberArm) / Kx;
        const double Ky = std::max(1.0, b.k_translation.y());
        toe_rad += w_steer * 0.15 * (Fy / Ky) * kToeArm;
    }
    if (Fz > 1.0) {
        const double sc = std::min(1.0, 5000.0 / Fz);
        toe_rad *= sc;
        camber_rad *= sc;
    }
}

CornerDaeParams corner_dae_params(SuspensionTopology& topo) {
    ensure_default_bushings(topo);
    CornerDaeParams p;
    for (const auto& b : topo.bushings) {
        p.k_toe += bushing_steer_weight(b) * std::max(1.0, b.k_rotation.z());
        p.k_camber += bushing_camber_weight(b) * std::max(1.0, b.k_rotation.x());
        p.c_toe += bushing_steer_weight(b) * std::max(1.0, b.c_rotation.z());
        p.c_camber += bushing_camber_weight(b) * std::max(1.0, b.c_rotation.x());
    }
    p.k_toe = std::max(1.0, p.k_toe);
    p.k_camber = std::max(1.0, p.k_camber);
    constexpr double kLeverToe = 0.22;
    constexpr double kLeverCamber = 0.18;
    for (const auto& body : topo.bodies) {
        if (body.id == "chassis" || body.mass <= 0.0) continue;
        p.m_toe += body.mass * kLeverToe * kLeverToe;
        p.m_camber += body.mass * kLeverCamber * kLeverCamber;
    }
    p.m_toe = std::max(0.15, p.m_toe);
    p.m_camber = std::max(0.15, p.m_camber);
    return p;
}

void solve_quasi_static_compliance(SuspensionTopology& topo, const WheelLoad& load) {
    ensure_default_bushings(topo);
    double toe_rad = 0.0, camber_rad = 0.0;
    compliance_targets_rad(topo, load, toe_rad, camber_rad);
    topo.compliance_toe_deg = toe_rad * 180.0 / M_PI;
    topo.compliance_camber_deg = camber_rad * 180.0 / M_PI;
    topo.toe_deg += topo.compliance_toe_deg;
    topo.camber_deg += topo.compliance_camber_deg;
}

}  // namespace vdsim::mb
