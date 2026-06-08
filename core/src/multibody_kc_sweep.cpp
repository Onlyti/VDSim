#include "vdsim/multibody.hpp"
#include "vdsim/suspension.hpp"

#include <cmath>

namespace vdsim::mb {

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

std::vector<double> linspace(double a, double b, int n) {
    std::vector<double> out;
    if (n < 1) return out;
    out.reserve(static_cast<std::size_t>(n));
    if (n == 1) {
        out.push_back(a);
        return out;
    }
    const double h = (b - a) / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) out.push_back(a + h * static_cast<double>(i));
    return out;
}

KcSweepSample sample_from_kin(const ISuspensionKinematics::Output& o, double abscissa) {
    KcSweepSample s;
    s.abscissa = abscissa;
    s.toe_deg = o.toe * kRadToDeg;
    s.camber_deg = o.camber * kRadToDeg;
    s.caster_deg = o.caster * kRadToDeg;
    s.track_mm = o.track_change * 1000.0;
    return s;
}

}  // namespace

KcSweepResult run_kc_sweep(const SuspensionTopology& topo,
                           const KcSweepParams& params) {
    if (topo.kin_yaml_path.empty())
        throw std::runtime_error("mb: kc sweep requires kin_yaml_path");
    auto kin = create_native_kinematics_from_yaml(topo.kin_yaml_path);
    auto solver = create_kinematic_solver();
    KcSweepResult out;

    for (double travel : linspace(params.travel_min_m, params.travel_max_m,
                                  params.travel_n)) {
        const auto o = kin->compute(travel, 0.0);
        out.travel.push_back(sample_from_kin(o, travel * 1000.0));
    }

    for (double rack : linspace(params.steer_rack_min_m, params.steer_rack_max_m,
                                params.steer_n)) {
        const auto o = kin->compute(0.0, rack);
        out.steer.push_back(sample_from_kin(o, rack * 1000.0));
    }

    for (double fy : linspace(params.fy_min_n, params.fy_max_n, params.fy_n)) {
        SuspensionTopology t = topo;
        solver->forward_kinematics(t, 0.0, 0.0);
        WheelLoad load;
        load.force_world.y() = fy;
        load.force_world.z() = params.fz_nominal_n;
        solver->quasi_static_compliance(t, load);
        KcSweepSample s;
        s.abscissa = fy;
        s.toe_deg = t.toe_deg;
        s.camber_deg = t.camber_deg;
        s.compliance_toe_deg = t.compliance_toe_deg;
        s.compliance_camber_deg = t.compliance_camber_deg;
        out.compliance_fy.push_back(s);
    }

    return out;
}

}  // namespace vdsim::mb
