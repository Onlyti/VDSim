// Pacejka Magic Formula 1996 (simplified, decoupled long/lat).
//
//   F = D * sin(C * atan(B*s - E*(B*s - atan(B*s))))
//
// Where D = peak force = Fz * mu * D_param.
// Lateral takes a leading minus sign so that the linear-region slope
// matches F_y = -C_alpha * alpha (ISO 8855 sign convention).
//
// Combined slip (friction-ellipse rescale):
//   (Fx_pure / Fx_max)^2 + (Fy_pure / Fy_max)^2 > 1
//      => (Fx, Fy) scaled by 1/ratio
//   Only one of kappa/alpha nonzero -> ratio <= 1 -> no change (legacy tests preserved).
//
// Self-aligning moment:
//   Mz = -t_p(alpha) * Fy_combined
//   t_p(alpha) = t_p_0 * cos(atan(alpha / alpha_falloff))
//             ~ t_p_0 / sqrt(1 + (alpha/alpha_falloff)^2)
//   alpha_falloff > 0 ; for alpha -> 0 we have t_p -> t_p_0.

#include "vdsim/interfaces.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace vdsim {

namespace {

inline double pacejka_form(double B, double C, double D, double E, double s) {
    const double t   = B * s;
    const double phi = t - E * (t - std::atan(t));
    return D * std::sin(C * std::atan(phi));
}

class PacejkaMF96 final : public ITireModel {
public:
    void initialize(const TireParams& tp) override { tp_ = tp; }

    Output compute(const Input& in) const noexcept override {
        Output out;
        const double Fz = std::max(0.0, in.Fz);
        if (Fz < 1.0) return out;

        const double mu_x = in.mu_long * tp_.mu_nominal;
        const double mu_y = in.mu_lat  * tp_.mu_nominal;
        const double Fx_max = tp_.D_long * Fz * mu_x;
        const double Fy_max = tp_.D_lat  * Fz * mu_y;

        // Pure-slip forces.
        const double Fx_pure =  pacejka_form(tp_.B_long, tp_.C_long, Fx_max,
                                              tp_.E_long, in.kappa);
        const double Fy_pure = -pacejka_form(tp_.B_lat,  tp_.C_lat,  Fy_max,
                                              tp_.E_lat,  in.alpha);

        double Fx = Fx_pure, Fy = Fy_pure;

        // Friction-ellipse rescale.
        if (tp_.combined_slip_enabled && Fx_max > 0.0 && Fy_max > 0.0) {
            const double rx = Fx_pure / Fx_max;
            const double ry = Fy_pure / Fy_max;
            const double ratio_sq = rx * rx + ry * ry;
            if (ratio_sq > 1.0) {
                const double scale = 1.0 / std::sqrt(ratio_sq);
                Fx = Fx_pure * scale;
                Fy = Fy_pure * scale;
            }
        }

        // Self-aligning moment with pneumatic trail falloff.
        const double tp0   = tp_.pneumatic_trail;
        const double a_fo  = (tp_.trail_falloff_alpha > 1e-6)
                             ? tp_.trail_falloff_alpha : 1e-6;
        const double trail = tp0 / std::sqrt(1.0 + (in.alpha / a_fo) *
                                                    (in.alpha / a_fo));
        // Camber thrust (linear): Fy_camber = -C_gamma · gamma · Fz · mu_lat
        // Sign: positive camber (top of tire leans into +y) generates +y force
        //       which conventionally is negative Fy (toward wheel forward).
        const double Fy_camber = -tp_.camber_stiffness * in.gamma * Fz * mu_y;
        out.Fx = Fx;
        out.Fy = Fy + Fy_camber;
        out.Mz = -trail * Fy;        // Mz from cornering Fy only (camber excluded for PoC)
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
