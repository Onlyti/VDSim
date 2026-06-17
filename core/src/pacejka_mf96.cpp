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
    Output compute(const Input& in) const noexcept override {
        const TireParams& tp_ = params_;
        Output out;
        const double Fz = std::max(0.0, in.Fz);
        if (Fz < 1.0) return out;

        // Load-sensitive μ:  μ_eff = μ_nominal · (1 − ls · (Fz/Fz_nom − 1))
        // Floored at 0.3 · μ_nominal to keep numerics sane.
        const double Fz_n  = std::max(1.0, tp_.Fz_nominal);
        const double dfz   = Fz / Fz_n - 1.0;
        const double mu_e  = std::max(0.3 * tp_.mu_nominal,
                                       tp_.mu_nominal * (1.0 - tp_.load_sensitivity * dfz));
        const double mu_x = in.mu_long * mu_e;
        const double mu_y = in.mu_lat  * mu_e;
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
        // Camber thrust (linear): Fy_camber = +C_gamma · gamma · Fz · mu_lat.
        // Sign: ISO 8855 (y = left). Positive inclination (top of tire toward +y)
        // makes the tire roll toward +y, so the camber thrust is +y, i.e. a POSITIVE
        // Fy in this model's basis (where the slip force Fy_pure = -form(alpha) already
        // puts a +y force at positive Fy). The previous leading minus contradicted that
        // basis and pushed camber thrust the wrong way.
        const double Fy_camber = tp_.camber_stiffness * in.gamma * Fz * mu_y;
        // Camber aligning-moment contribution (small, linear), tied to the same
        // stiffness with a fixed pneumatic-trail fraction. Sign kept consistent with
        // the corrected camber thrust; magnitude is a placeholder pending K&C/test data.
        const double Mz_camber = tp_.pneumatic_trail * 0.25 *
                                  tp_.camber_stiffness * in.gamma * Fz * mu_y;
        out.Fx = Fx;
        out.Fy = Fy + Fy_camber;
        out.Mz = -trail * Fy + Mz_camber;
        out.mu_peak = std::max(tp_.D_long * mu_x, tp_.D_lat * mu_y);
        return out;
    }
};

}  // namespace

std::unique_ptr<ITireModel> create_pacejka_mf96() {
    return std::make_unique<PacejkaMF96>();
}

}  // namespace vdsim
