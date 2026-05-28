// Pacejka Magic Formula 1996 (simplified, decoupled long/lat).
//
//   F = D * sin(C * atan(B*s - E*(B*s - atan(B*s))))
//
// Where D = peak force = Fz * mu * D_param.
// Lateral takes a leading minus sign so that the linear-region slope
// matches F_y = -C_alpha * alpha (ISO 8855 sign convention).

#include "vdsim/interfaces.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace vdsim {

namespace {

class PacejkaMF96 final : public ITireModel {
public:
    void initialize(const TireParams& tp) override { tp_ = tp; }

    Output compute(const Input& in) const noexcept override {
        Output out;  // zero-initialized

        const double Fz = std::max(0.0, in.Fz);
        if (Fz < 1.0) return out;   // wheel effectively off the ground

        const double mu_x = in.mu_long * tp_.mu_nominal;
        const double mu_y = in.mu_lat  * tp_.mu_nominal;

        const double Dx = tp_.D_long * Fz * mu_x;
        const double Dy = tp_.D_lat  * Fz * mu_y;

        // Longitudinal: positive kappa (drive) -> positive Fx.
        {
            const double s   = in.kappa;
            const double t   = tp_.B_long * s;
            const double phi = t - tp_.E_long * (t - std::atan(t));
            out.Fx = Dx * std::sin(tp_.C_long * std::atan(phi));
        }
        // Lateral: positive alpha -> negative Fy (restoring).
        {
            const double s   = in.alpha;
            const double t   = tp_.B_lat * s;
            const double phi = t - tp_.E_lat * (t - std::atan(t));
            out.Fy = -Dy * std::sin(tp_.C_lat * std::atan(phi));
        }
        // Aligning moment: not modeled in PoC.
        return out;
    }

private:
    TireParams tp_;
};

}  // namespace

std::unique_ptr<ITireModel> create_pacejka_mf96() {
    return std::make_unique<PacejkaMF96>();
}

}  // namespace vdsim
