// Full Magic Formula (Pacejka 2002 / MF-Tyre 5.2) tire model implementation.
//
// Equations follow Pacejka, "Tire and Vehicle Dynamics" 2nd ed., Ch. 4.3.2
// (the steady-state Magic Formula with combined slip).  Turn-slip, inflation
// pressure and thermal terms are omitted (absent from this coefficient set).
//
// Sign convention: the .tir coefficients are assumed ISO (TYRESIDE-consistent),
// so cornering stiffness Kya carries its sign from PKY1; no manual flip is
// applied.  Surface friction scaling is folded in via Input.mu_long/mu_lat.
//
// Coefficient *values* are loaded at runtime from an external .tir and are NOT
// embedded here.
#include "vdsim/magic_formula.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace vdsim {
namespace {

constexpr double kEps = 1e-6;

inline double sgn(double x) { return (x > 0.0) - (x < 0.0); }
inline double clamp_hi(double x, double hi) { return x > hi ? hi : x; }

// One Magic Formula sine branch: D sin(C atan(Bx - E(Bx - atan Bx))).
inline double mf_sin(double B, double C, double D, double E, double x) {
    const double bx = B * x;
    return D * std::sin(C * std::atan(bx - E * (bx - std::atan(bx))));
}
// One Magic Formula cosine branch (used for weighting / pneumatic trail).
inline double mf_cos(double B, double C, double E, double x) {
    const double bx = B * x;
    return std::cos(C * std::atan(bx - E * (bx - std::atan(bx))));
}

class MagicFormulaTire final : public ITireModel {
public:
    explicit MagicFormulaTire(MFCoeffs c) : c_(std::move(c)) {
        Fz0_  = c_.g("FNOMIN", 4000.0);
        R0_   = c_.g("UNLOADED_RADIUS", 0.3);
        LFZO_ = c_.g("LFZO", 1.0);
    }

    void on_initialize(const TireParams&) override {}  // coefficients come from .tir
                                                       // (Re/crown/transient via base params_)

    Output compute(const Input& in) const noexcept override {
        Output out;
        const double Fz = std::max(0.0, in.Fz);
        if (Fz < 1.0) return out;

        const double Fz0p = std::max(1.0, LFZO_ * Fz0_);
        const double dfz  = (Fz - Fz0p) / Fz0p;
        const double k    = in.kappa;
        const double a    = in.alpha;
        const double g     = in.gamma;
        const double g2    = g * g;

        // Surface mu scaling folded into the MF mu scaling factors.
        const double Lmux = c_.g("LMUX", 1.0) * std::max(0.0, in.mu_long);
        const double Lmuy = c_.g("LMUY", 1.0) * std::max(0.0, in.mu_lat);

        // ---------- Pure longitudinal Fx0 ----------
        const double SHx = (c_.g("PHX1", 0) + c_.g("PHX2", 0) * dfz) * c_.g("LHX", 1);
        const double kx  = k + SHx;
        const double Cx  = c_.g("PCX1", 1.65) * c_.g("LCX", 1);
        const double mux = (c_.g("PDX1", 1) + c_.g("PDX2", 0) * dfz) * (1.0 - c_.g("PDX3", 0) * g2) * Lmux;
        const double Dx  = mux * Fz;
        double Ex = (c_.g("PEX1", 0) + c_.g("PEX2", 0) * dfz + c_.g("PEX3", 0) * dfz * dfz)
                    * (1.0 - c_.g("PEX4", 0) * sgn(kx)) * c_.g("LEX", 1);
        Ex = clamp_hi(Ex, 1.0);
        const double Kxk = Fz * (c_.g("PKX1", 20) + c_.g("PKX2", 0) * dfz)
                           * std::exp(c_.g("PKX3", 0) * dfz) * c_.g("LKX", 1);
        const double Bx  = Kxk / (Cx * Dx + kEps);
        const double SVx = Fz * (c_.g("PVX1", 0) + c_.g("PVX2", 0) * dfz) * c_.g("LVX", 1) * Lmux;
        const double Fx0 = mf_sin(Bx, Cx, Dx, Ex, kx) + SVx;

        // ---------- Pure lateral Fy0 ----------
        const double SHy = (c_.g("PHY1", 0) + c_.g("PHY2", 0) * dfz) * c_.g("LHY", 1)
                           + c_.g("PHY3", 0) * g * c_.g("LGAY", 1);
        const double ay  = a + SHy;
        const double Cy  = c_.g("PCY1", 1.3) * c_.g("LCY", 1);
        const double muy = (c_.g("PDY1", 1) + c_.g("PDY2", 0) * dfz) * (1.0 - c_.g("PDY3", 0) * g2) * Lmuy;
        const double Dy  = muy * Fz;
        double Ey = (c_.g("PEY1", 0) + c_.g("PEY2", 0) * dfz)
                    * (1.0 - (c_.g("PEY3", 0) + c_.g("PEY4", 0) * g) * sgn(ay)) * c_.g("LEY", 1);
        Ey = clamp_hi(Ey, 1.0);
        const double Kya = c_.g("PKY1", -15) * Fz0p
                           * std::sin(2.0 * std::atan(Fz / (c_.g("PKY2", 2) * Fz0p)))
                           * (1.0 - c_.g("PKY3", 0) * std::fabs(g)) * c_.g("LKY", 1);
        const double By  = Kya / (Cy * Dy + kEps);  // sign carried by Kya (ISO: PKY1<0)
        const double SVy = Fz * ((c_.g("PVY1", 0) + c_.g("PVY2", 0) * dfz) * c_.g("LVY", 1)
                           + (c_.g("PVY3", 0) + c_.g("PVY4", 0) * dfz) * g * c_.g("LGAY", 1)) * Lmuy;
        const double Fy0 = mf_sin(By, Cy, Dy, Ey, ay) + SVy;

        // ---------- Combined longitudinal (weighting Gxa) ----------
        const double SHxa = c_.g("RHX1", 0);
        const double aS   = a + SHxa;
        const double Bxa  = c_.g("RBX1", 10) * mf_cos_arg(c_.g("RBX2", 0) * k) * c_.g("LXAL", 1);
        const double Cxa  = c_.g("RCX1", 1);
        double Exa = c_.g("REX1", 0) + c_.g("REX2", 0) * dfz; Exa = clamp_hi(Exa, 1.0);
        const double Gxa0 = mf_cos(Bxa, Cxa, Exa, SHxa);
        const double Gxa  = std::fabs(Gxa0) > kEps
                            ? mf_cos(Bxa, Cxa, Exa, aS) / Gxa0 : 1.0;
        const double Fx = Gxa * Fx0;

        // ---------- Combined lateral (weighting Gyk + SVyk) ----------
        const double SHyk = c_.g("RHY1", 0) + c_.g("RHY2", 0) * dfz;
        const double kS   = k + SHyk;
        const double Byk  = c_.g("RBY1", 10) * mf_cos_arg(c_.g("RBY2", 0) * (a - c_.g("RBY3", 0))) * c_.g("LYKA", 1);
        const double Cyk  = c_.g("RCY1", 1);
        double Eyk = c_.g("REY1", 0) + c_.g("REY2", 0) * dfz; Eyk = clamp_hi(Eyk, 1.0);
        const double Gyk0 = mf_cos(Byk, Cyk, Eyk, SHyk);
        const double Gyk  = std::fabs(Gyk0) > kEps
                            ? mf_cos(Byk, Cyk, Eyk, kS) / Gyk0 : 1.0;
        const double DVyk = muy * Fz * (c_.g("RVY1", 0) + c_.g("RVY2", 0) * dfz + c_.g("RVY3", 0) * g)
                            * mf_cos_arg(c_.g("RVY4", 0) * a);
        const double SVyk = DVyk * std::sin(c_.g("RVY5", 0) * std::atan(c_.g("RVY6", 0) * k)) * c_.g("LVYKA", 1);
        const double Fy = Gyk * Fy0 + SVyk;

        // ---------- Aligning moment Mz (combined) ----------
        // Equivalent slips couple longitudinal slip into the trail/residual.
        const double Kr  = (std::fabs(Kya) > kEps) ? (Kxk / Kya) : 0.0;
        const double SHt = c_.g("QHZ1", 0) + c_.g("QHZ2", 0) * dfz
                           + (c_.g("QHZ3", 0) + c_.g("QHZ4", 0) * dfz) * g;
        const double at  = a + SHt;
        const double at_eq = sgn(at) * std::sqrt(at * at + Kr * Kr * k * k);
        const double SHf = SHy + (std::fabs(Kya) > kEps ? SVy / Kya : 0.0);
        const double ar  = a + SHf;
        const double ar_eq = sgn(ar) * std::sqrt(ar * ar + Kr * Kr * k * k);

        const double Bt = (c_.g("QBZ1", 10) + c_.g("QBZ2", 0) * dfz + c_.g("QBZ3", 0) * dfz * dfz)
                          * (1.0 + c_.g("QBZ4", 0) * g + c_.g("QBZ5", 0) * std::fabs(g))
                          * c_.g("LKY", 1) / std::max(kEps, c_.g("LMUY", 1));
        const double Ct = c_.g("QCZ1", 1.1);
        const double Dt = Fz * (c_.g("QDZ1", 0.1) + c_.g("QDZ2", 0) * dfz)
                          * (1.0 + c_.g("QDZ3", 0) * std::fabs(g) + c_.g("QDZ4", 0) * g2)
                          * (R0_ / Fz0p) * c_.g("LTR", 1);
        double Et = (c_.g("QEZ1", 0) + c_.g("QEZ2", 0) * dfz + c_.g("QEZ3", 0) * dfz * dfz)
                    * (1.0 + (c_.g("QEZ4", 0) + c_.g("QEZ5", 0) * g) * (2.0 / M_PI) * std::atan(Bt * Ct * at));
        Et = clamp_hi(Et, 1.0);
        const double trail = Dt * mf_cos(Bt, Ct, Et, at_eq) * std::cos(a);

        const double Br = c_.g("QBZ9", 0) * c_.g("LKY", 1) / std::max(kEps, c_.g("LMUY", 1))
                          + c_.g("QBZ10", 0) * By * Cy;
        const double Dr = Fz * ((c_.g("QDZ6", 0) + c_.g("QDZ7", 0) * dfz) * c_.g("LRES", 1)
                          + (c_.g("QDZ8", 0) + c_.g("QDZ9", 0) * dfz) * g) * R0_ * Lmuy;
        const double Mzr = Dr * std::cos(std::atan(Br * ar_eq)) * std::cos(a);

        // s * Fx term (lateral force moment arm from longitudinal force).
        const double s = R0_ * (c_.g("SSZ1", 0) + c_.g("SSZ2", 0) * (Fy / Fz0p)
                          + (c_.g("SSZ3", 0) + c_.g("SSZ4", 0) * dfz) * g) * c_.g("LS", 1);

        // Lateral force feeding the trail uses the combined Fy minus the
        // kappa-induced vertical shift (so pure SVyk does not bias the trail).
        const double Fy_trail = Fy - SVyk;
        const double Mz = -trail * Fy_trail + Mzr + s * Fx;

        out.Fx = Fx;
        out.Fy = Fy;
        out.Mz = Mz;
        return out;
    }

private:
    // cos(atan(x)) = 1/sqrt(1+x^2) — used inside weighting B-factors.
    static double mf_cos_arg(double x) { return std::cos(std::atan(x)); }

    MFCoeffs c_;
    double Fz0_{4000.0}, R0_{0.3}, LFZO_{1.0};
};

}  // namespace

MFCoeffs parse_tir(const std::string& tir_path) {
    std::ifstream in(tir_path);
    if (!in) throw std::runtime_error("parse_tir: cannot open '" + tir_path + "'");

    MFCoeffs c;
    std::string line;
    while (std::getline(in, line)) {
        // Strip comments ('$' and '!') and trailing CR.
        auto cut = line.find_first_of("$!");
        if (cut != std::string::npos) line = line.substr(0, cut);
        if (!line.empty() && line.back() == '\r') line.pop_back();

        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;             // section header or blank
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // Trim + uppercase key.
        auto trim = [](std::string& s) {
            const auto b = s.find_first_not_of(" \t");
            const auto e = s.find_last_not_of(" \t");
            s = (b == std::string::npos) ? "" : s.substr(b, e - b + 1);
        };
        trim(key); trim(val);
        if (key.empty() || val.empty()) continue;
        for (auto& ch : key) ch = static_cast<char>(std::toupper(ch));

        // Parse numeric value; skip string entries (e.g. FILE_TYPE = 'tir').
        try {
            size_t pos = 0;
            const double num = std::stod(val, &pos);
            c.p[key] = num;
        } catch (...) {
            // non-numeric value -> ignore
        }
    }
    return c;
}

std::unique_ptr<ITireModel> create_magic_formula_tire(const MFCoeffs& coeffs) {
    return std::make_unique<MagicFormulaTire>(coeffs);
}

std::unique_ptr<ITireModel> create_magic_formula_tire_from_tir(const std::string& tir_path) {
    return std::make_unique<MagicFormulaTire>(parse_tir(tir_path));
}

std::unique_ptr<ITireModel> create_tire_from_params(const TireParams& tp) {
    if (tp.backend == "magic_formula" || tp.backend == "mf2002") {
        if (tp.tir_path.empty())
            throw std::runtime_error(
                "tire backend '" + tp.backend + "' requires a .tir path (tir_path)");
        return create_magic_formula_tire_from_tir(tp.tir_path);
    }
    if (tp.backend == "linear") return create_linear_tire();
    return create_pacejka_mf96();   // "mf96" / default
}

}  // namespace vdsim
