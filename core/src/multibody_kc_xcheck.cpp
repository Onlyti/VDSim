#include "vdsim/multibody.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace vdsim::mb {

namespace {

int nearest_zero_index(const std::vector<KcSweepSample>& samples) {
    if (samples.empty()) return 0;
    int best = 0;
    double best_d = std::abs(samples[0].abscissa);
    for (int i = 1; i < static_cast<int>(samples.size()); ++i) {
        const double d = std::abs(samples[i].abscissa);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

template <typename Getter>
double central_slope_at_zero(const std::vector<KcSweepSample>& samples, Getter get) {
    if (samples.size() < 3) return 0.0;
    const int i0 = nearest_zero_index(samples);
    if (i0 <= 0 || i0 >= static_cast<int>(samples.size()) - 1) return 0.0;
    const double dx = samples[static_cast<std::size_t>(i0 + 1)].abscissa
                    - samples[static_cast<std::size_t>(i0 - 1)].abscissa;
    if (std::abs(dx) < 1e-12) return 0.0;
    const double yp = get(samples[static_cast<std::size_t>(i0 + 1)]);
    const double ym = get(samples[static_cast<std::size_t>(i0 - 1)]);
    return (yp - ym) / dx;
}

bool metric_pass(double ref, double cand, double rtol, double atol) {
    if (std::abs(ref) < atol && std::abs(cand) < atol) return true;
    const double denom = std::max(atol, std::abs(ref));
    return std::abs(cand - ref) / denom <= rtol;
}

void add_delta(KcXcheckReport& rep, const std::string& name,
               double ref, double cand, double rtol, double atol) {
    KcMetricDelta d;
    d.name = name;
    d.reference = ref;
    d.candidate = cand;
    const double denom = std::max(atol, std::abs(ref));
    d.rel_error = std::abs(cand - ref) / denom;
    d.ok = metric_pass(ref, cand, rtol, atol);
    rep.deltas.push_back(d);
    rep.all_ok = rep.all_ok && d.ok;
}

}  // namespace

KcMetrics compute_kc_metrics(const KcSweepResult& sweep) {
    KcMetrics m;
    m.toe_gain_travel_deg_per_mm =
        central_slope_at_zero(sweep.travel,
                              [](const KcSweepSample& s) { return s.toe_deg; });
    m.camber_gain_travel_deg_per_mm =
        central_slope_at_zero(sweep.travel,
                              [](const KcSweepSample& s) { return s.camber_deg; });
    m.track_gain_travel_mm_per_mm =
        central_slope_at_zero(sweep.travel,
                              [](const KcSweepSample& s) { return s.track_mm; });
    m.toe_gain_steer_deg_per_mm =
        central_slope_at_zero(sweep.steer,
                              [](const KcSweepSample& s) { return s.toe_deg; });
    m.caster_gain_steer_deg_per_mm =
        central_slope_at_zero(sweep.steer,
                              [](const KcSweepSample& s) { return s.caster_deg; });
    return m;
}

KcXcheckReport compare_kc_metrics(const KcMetrics& reference,
                                  const KcMetrics& candidate,
                                  double rtol, double atol) {
    KcXcheckReport rep;
    rep.all_ok = true;
    add_delta(rep, "toe_gain_travel_deg_per_mm",
              reference.toe_gain_travel_deg_per_mm,
              candidate.toe_gain_travel_deg_per_mm, rtol, atol);
    add_delta(rep, "camber_gain_travel_deg_per_mm",
              reference.camber_gain_travel_deg_per_mm,
              candidate.camber_gain_travel_deg_per_mm, rtol, atol);
    add_delta(rep, "track_gain_travel_mm_per_mm",
              reference.track_gain_travel_mm_per_mm,
              candidate.track_gain_travel_mm_per_mm, rtol, atol);
    add_delta(rep, "toe_gain_steer_deg_per_mm",
              reference.toe_gain_steer_deg_per_mm,
              candidate.toe_gain_steer_deg_per_mm, rtol, atol);
    add_delta(rep, "caster_gain_steer_deg_per_mm",
              reference.caster_gain_steer_deg_per_mm,
              candidate.caster_gain_steer_deg_per_mm, rtol, atol);
    return rep;
}

KcXcheckReport run_kc_xcheck(const std::string& reference_yaml,
                             const std::string& candidate_yaml,
                             double rtol,
                             const KcSweepParams& params) {
    auto ref_topo = SuspensionTopology::from_yaml(reference_yaml);
    auto cand_topo = SuspensionTopology::from_yaml(candidate_yaml);
    const auto ref_sweep = run_kc_sweep(ref_topo, params);
    const auto cand_sweep = run_kc_sweep(cand_topo, params);
    return compare_kc_metrics(compute_kc_metrics(ref_sweep),
                              compute_kc_metrics(cand_sweep), rtol);
}

}  // namespace vdsim::mb
