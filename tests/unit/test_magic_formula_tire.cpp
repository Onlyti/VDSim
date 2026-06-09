// T1 — Magic Formula (MF2002 / .tir) evaluator sanity.
//
// The .tir parser + MF2002 evaluator already exist (magic_formula.hpp). This is
// the first T1 coverage: parse a *synthetic, public* .tir written at runtime
// (no .tir is committed — repo .gitignore blocks *.tir and real coefficients are
// confidential) and check the pure/combined-slip forces are physically sane.

#include "vdsim/interfaces.hpp"
#include "vdsim/magic_formula.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {

// Synthetic, representative passenger-car coefficients (NOT measured data).
const char* kSyntheticTir = R"TIR(
$ synthetic public coefficients for testing only
[MODEL]
[DIMENSION]
UNLOADED_RADIUS = 0.31
[VERTICAL]
FNOMIN = 4000
[LONGITUDINAL_COEFFICIENTS]
PCX1 = 1.65
PDX1 = 1.20
PDX2 = -0.05
PEX1 = 0.50
PKX1 = 20.0
[LATERAL_COEFFICIENTS]
PCY1 = 1.30
PDY1 = 1.10
PDY2 = -0.05
PEY1 = -0.50
PKY1 = -15.0
[ALIGNING_COEFFICIENTS]
QBZ1 = 6.0
QDZ1 = 0.10
)TIR";

fs::path write_synthetic_tir() {
    const auto p = fs::temp_directory_path() /
        ("vdsim_synth_" + std::to_string(static_cast<long>(::getpid())) + ".tir");
    std::ofstream(p) << kSyntheticTir;
    return p;
}

vdsim::ITireModel::Input slip_input(double kappa, double alpha, double Fz) {
    vdsim::ITireModel::Input in;
    in.Fz = Fz;
    in.kappa = kappa;
    in.alpha = alpha;
    in.mu_long = 1.0;
    in.mu_lat = 1.0;
    in.Vx_wheel = 15.0;
    return in;
}

}  // namespace

TEST(Mf2002Catalog, ParsesSyntheticTir) {
    const auto p = write_synthetic_tir();
    const vdsim::MFCoeffs c = vdsim::parse_tir(p.string());
    EXPECT_TRUE(c.has("FNOMIN"));
    EXPECT_DOUBLE_EQ(c.g("FNOMIN", 0.0), 4000.0);
    EXPECT_DOUBLE_EQ(c.g("PKX1", 0.0), 20.0);
    EXPECT_DOUBLE_EQ(c.g("missing", -1.0), -1.0);   // default for absent key
    fs::remove(p);
}

TEST(Mf2002Catalog, SampleTirRunsAndIsSane) {
    const auto p = write_synthetic_tir();
    auto tire = vdsim::create_magic_formula_tire_from_tir(p.string());
    fs::remove(p);
    ASSERT_NE(tire, nullptr);

    const double Fz = 4000.0, muFz = 1.0 * Fz;

    // Zero slip -> ~zero force.
    {
        const auto o = tire->compute(slip_input(0.0, 0.0, Fz));
        EXPECT_LT(std::abs(o.Fx), 50.0);
        EXPECT_LT(std::abs(o.Fy), 50.0);
    }
    // Pure longitudinal: drive slip -> forward force, bounded by ~mu*Fz.
    {
        const auto o = tire->compute(slip_input(0.10, 0.0, Fz));
        EXPECT_GT(o.Fx, 500.0);
        EXPECT_LT(o.Fx, 1.3 * muFz);
        EXPECT_LT(std::abs(o.Fy), 200.0);          // little lateral at alpha=0
    }
    // Pure lateral: slip angle -> lateral force develops, bounded.
    {
        const auto o = tire->compute(slip_input(0.0, 0.10, Fz));
        EXPECT_GT(std::abs(o.Fy), 500.0);
        EXPECT_LT(std::abs(o.Fy), 1.3 * muFz);
    }
    // Saturation: large longitudinal slip stays within the friction budget.
    {
        const auto o = tire->compute(slip_input(0.8, 0.0, Fz));
        EXPECT_LT(std::abs(o.Fx), 1.5 * muFz);
    }
    // Below the load floor -> no force.
    {
        const auto o = tire->compute(slip_input(0.1, 0.1, 0.5));
        EXPECT_DOUBLE_EQ(o.Fx, 0.0);
        EXPECT_DOUBLE_EQ(o.Fy, 0.0);
    }
}

// T1.2 — backend dispatch from TireParams.
TEST(Mf2002Catalog, DispatchByBackend) {
    using namespace vdsim;
    { TireParams tp;                       // default -> mf96
      EXPECT_NE(create_tire_from_params(tp), nullptr); }
    { TireParams tp; tp.backend = "linear";
      EXPECT_NE(create_tire_from_params(tp), nullptr); }
    { TireParams tp; tp.backend = "magic_formula";   // needs a .tir path
      EXPECT_THROW(create_tire_from_params(tp), std::runtime_error); }
    { const auto p = write_synthetic_tir();
      TireParams tp; tp.backend = "magic_formula"; tp.tir_path = p.string();
      auto t = create_tire_from_params(tp);
      fs::remove(p);
      ASSERT_NE(t, nullptr);
      EXPECT_GT(t->compute(slip_input(0.1, 0.0, 4000.0)).Fx, 500.0); }
}

// T1.4 — the MF2002 evaluator does its own combined slip (Gxa/Gyk), so the host
// must not re-clip it with the circular friction ellipse.
TEST(Mf2002Catalog, CombinedSlipBypassesHostEllipse) {
    using namespace vdsim;
    // Predicate: MF2002 + LuGre (combined) bypass the host ellipse; MF96 does not.
    { TireParams tp; tp.backend = "magic_formula"; EXPECT_TRUE(tp.model_provides_combined_slip()); }
    { TireParams tp; tp.backend = "mf2002";        EXPECT_TRUE(tp.model_provides_combined_slip()); }
    { TireParams tp; tp.lugre.enabled = true;      EXPECT_TRUE(tp.model_provides_combined_slip()); }
    // Pure MF96 (LuGre off, default backend) couples via the host ellipse instead.
    { TireParams tp; tp.lugre.enabled = false;     EXPECT_FALSE(tp.model_provides_combined_slip()); }
    { TireParams tp; tp.lugre.enabled = false; tp.backend = "magic_formula";
      tp.combined_slip_enabled = false;            EXPECT_FALSE(tp.model_provides_combined_slip()); }

    // MF2002 peak longitudinal force legitimately exceeds mu_nominal*Fz (its mu is
    // in PDX1), so a circular host clip at mu_nominal*Fz would corrupt it.
    const auto p = write_synthetic_tir();
    auto tire = create_magic_formula_tire_from_tir(p.string());
    fs::remove(p);
    const double Fz = 4000.0, muFz = 1.0 * Fz;
    double fx_peak = 0.0;
    for (double k = 0.02; k <= 0.4; k += 0.02)
        fx_peak = std::max(fx_peak, tire->compute(slip_input(k, 0.0, Fz)).Fx);
    EXPECT_GT(fx_peak, muFz) << "MF2002 peak Fx must exceed mu_nominal*Fz (clip would corrupt)";
}

// A catalog/tire YAML selects the backend via from_yaml (the path cosim + batch
// use to load a tire). Proves a tire part can pick MF2002 from a runtime .tir.
TEST(Mf2002Catalog, YamlSelectsBackend) {
    using namespace vdsim;
    const auto tir = write_synthetic_tir();
    const auto yml = fs::temp_directory_path() /
        ("vdsim_tire_" + std::to_string(static_cast<long>(::getpid())) + ".yaml");
    {
        std::ofstream o(yml);
        o << "mu_nominal: 1.0\n"
          << "backend: magic_formula\n"
          << "tir_path: " << tir.string() << "\n";
    }
    const TireParams tp = TireParams::from_yaml(yml.string());
    fs::remove(yml);
    EXPECT_EQ(tp.backend, "magic_formula");
    EXPECT_EQ(tp.tir_path, tir.string());

    auto tire = create_tire_from_params(tp);
    fs::remove(tir);
    ASSERT_NE(tire, nullptr);
    EXPECT_GT(tire->compute(slip_input(0.1, 0.0, 4000.0)).Fx, 500.0);
}

// initialize() must swap the dynamics' tire to the requested backend and run.
TEST(Mf2002Catalog, L5UsesMagicFormulaBackend) {
    using namespace vdsim;
    const auto p = write_synthetic_tir();
    VehicleParams vp; vp.aero_drag_coeff = 0.0;
    TireParams tp; tp.backend = "magic_formula"; tp.tir_path = p.string();
    SolverParams sp; sp.stunt_physics = true; sp.max_substep_dt = 2e-4; sp.max_substeps = 16;
    auto dyn = create_stunt_dof();
    dyn->initialize(vp, tp, sp);     // <- backend swap happens here
    fs::remove(p);

    State s;
    s.position.z() = vp.cg_height;
    s.velocity.x() = 10.0;
    const double w = 10.0 / vp.wheel_radius_nominal;
    s.wheel_spin = {{w, w, w, w}};
    dyn->reset(s);

    auto ground = create_flat_ground(0.0, 1.0);
    CmdL4 cmd; cmd.throttle = 0.5;
    const ControlInput u = cmd;
    for (int i = 0; i < 2500; ++i) {
        ContactArray c;
        ground->query(dyn->state(), vp, c);
        dyn->step(u, c, 0.001);
    }
    EXPECT_TRUE(std::isfinite(dyn->state().velocity.x()));
    EXPECT_GT(dyn->state().velocity.x(), 10.3);   // accelerated under the MF2002 backend
}
