// Linear tire model:
//   Fx = C_kappa * kappa
//   Fy = -C_alpha * alpha          (ISO 8855 sign)
// Used for analytical validation and as a fallback at very low slip.

#include "vdsim/interfaces.hpp"

#include <algorithm>
#include <memory>

namespace vdsim {

namespace {

class LinearTire final : public ITireModel {
public:
    void on_initialize(const TireParams& tp) override {
        C_alpha_ = tp.cornering_stiffness;
        // Common approximation: longitudinal stiffness ~ 2x cornering stiffness.
        // Acceptable for PoC; refine when an explicit C_kappa is added.
        C_kappa_ = 2.0 * tp.cornering_stiffness;
    }

    Output compute(const Input& in) const noexcept override {
        Output out;
        out.Fx = C_kappa_ * in.kappa * in.mu_long;
        out.Fy = -C_alpha_ * in.alpha * in.mu_lat;
        out.Mz = 0.0;
        out.mu_peak = std::max(in.mu_long, in.mu_lat);
        return out;
    }

private:
    double C_alpha_ {0.0};
    double C_kappa_ {0.0};
};

}  // namespace

std::unique_ptr<ITireModel> create_linear_tire() {
    return std::make_unique<LinearTire>();
}

}  // namespace vdsim
