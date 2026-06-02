// Full Magic Formula tire demo + qualitative validation.
//
// Loads a Pacejka .tir (path given as argv[1]), checks the standalone tire
// evaluator (pure/combined slip shape, sign, load sensitivity, aligning Mz),
// then runs an Ld2 step-steer driven by the measured tire and compares it
// qualitatively against the default built-in tire.
//
// Prints qualitative PASS/FAIL/sign tokens only — no absolute force values or
// coefficients (a .tir may contain confidential measured data; keep the file
// out of the repository).
//
// Usage: vdsim_mf_tire_demo <path/to/tire.tir>
#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"
#include "vdsim/params.hpp"

#include <cmath>
#include <cstdio>

using namespace vdsim;

static ITireModel::Input mk(double Fz, double k, double a, double g) {
    ITireModel::Input in; in.Fz = Fz; in.kappa = k; in.alpha = a; in.gamma = g;
    in.mu_long = 1.0; in.mu_lat = 1.0; in.Vx_wheel = 20.0; return in;
}

static void validate_tire(const char* path) {
    auto t = create_magic_formula_tire_from_tir(path);
    auto c = parse_tir(path);
    const double Fz = c.g("FNOMIN", 4000.0);

    double peakFy = 0, aPk = 0;
    for (int i = 1; i <= 60; ++i) {
        double a = 0.005 * i, Fy = t->compute(mk(Fz, 0, a, 0)).Fy;
        if (std::fabs(Fy) > std::fabs(peakFy)) { peakFy = Fy; aPk = a; }
    }
    double peakFx = 0, kPk = 0;
    for (int i = 1; i <= 60; ++i) {
        double k = 0.005 * i, Fx = t->compute(mk(Fz, k, 0, 0)).Fx;
        if (std::fabs(Fx) > std::fabs(peakFx)) { peakFx = Fx; kPk = k; }
    }
    bool latISO = t->compute(mk(Fz, 0, 0.05, 0)).Fy < 0;
    bool lonPos = t->compute(mk(Fz, 0.05, 0, 0)).Fx > 0;
    double Fy_a  = std::fabs(t->compute(mk(Fz, 0.0,  0.10, 0)).Fy);
    double Fy_ak = std::fabs(t->compute(mk(Fz, 0.15, 0.10, 0)).Fy);
    auto pmu = [&](double fz){ double p=0; for(int i=1;i<=60;++i){double f=
        std::fabs(t->compute(mk(fz,0,0.005*i,0)).Fy); if(f>p)p=f;} return p/fz; };
    double Mz = t->compute(mk(Fz, 0, 0.03, 0)).Mz;

    std::printf("[tire]  coeffs parsed       : %zu\n", c.p.size());
    std::printf("[tire]  lateral ISO sign    : %s\n", latISO ? "PASS(+a->-Fy)" : "CHECK");
    std::printf("[tire]  lateral peak/sat    : %s\n", aPk < 0.295 ? "PASS" : "FAIL");
    std::printf("[tire]  long sign           : %s\n", lonPos ? "PASS(+k->+Fx)" : "CHECK");
    std::printf("[tire]  long peak/sat       : %s\n", kPk < 0.295 ? "PASS" : "FAIL");
    std::printf("[tire]  combined coupling   : %s\n", Fy_ak < Fy_a ? "PASS" : "FAIL");
    std::printf("[tire]  load sensitivity    : %s\n", pmu(2*Fz) < pmu(0.5*Fz) ? "PASS" : "FAIL");
    std::printf("[tire]  aligning Mz @ +a    : %s\n", Mz > 0 ? "+" : (Mz < 0 ? "-" : "0"));
}

static double step_steer(IVehicleDynamics& dyn) {
    VehicleParams vp; TireParams tp; SolverParams sp;
    dyn.initialize(vp, tp, sp);
    State s0; s0.velocity = {20.0, 0.0, 0.0};
    dyn.reset(s0);
    ContactArray contacts;
    for (auto& p : contacts) { p.is_valid = true; p.normal = {0,0,1}; p.mu_long = 1; p.mu_lat = 1; }
    CmdL4 cmd; cmd.steer_angle_wheel = 0.03;
    ControlInput u = cmd;
    for (int i = 0; i < 600; ++i) dyn.step(u, contacts, 0.005);
    return dyn.state().yaw_rate();
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path/to/tire.tir>\n", argv[0]);
        return 2;
    }
    validate_tire(argv[1]);

    auto def = create_seven_dof();
    auto mf  = create_seven_dof(create_magic_formula_tire_from_tir(argv[1]));
    double r_def = step_steer(*def);
    double r_mf  = step_steer(*mf);

    bool finite  = std::isfinite(r_mf);
    bool steady  = finite && r_mf > 1e-4;
    bool differs = std::isfinite(r_def) && steady &&
                   std::fabs(r_mf - r_def) / std::fabs(r_def) > 0.02;

    std::printf("[Ld2]   default step-steer  : %s\n",
                (std::isfinite(r_def) && r_def > 1e-4) ? "PASS(steady +r)" : "FAIL");
    std::printf("[Ld2]   MF finite (no NaN)  : %s\n", finite ? "PASS" : "FAIL");
    std::printf("[Ld2]   MF steady yaw (ISO) : %s\n", steady ? "PASS(+)" : "FAIL");
    std::printf("[Ld2]   MF vs default tire  : %s\n",
                differs ? "PASS(measured-driven)" : "(near-equal)");
    return 0;
}
