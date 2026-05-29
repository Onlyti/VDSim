// Split-mu accel demo: rear-left tire on ice (mu=0.2), rear-right on dry (mu=1.0).
// Compare 3 differential types: Open / Locked / LSD.

#include "vdsim/coordinate.hpp"
#include "vdsim/interfaces.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

vdsim::ContactArray split_mu_contacts() {
    vdsim::ContactArray c;
    for (auto& p : c) { p.is_valid = true; p.normal = vdsim::Vec3::UnitZ();
                       p.mu_long = 1.0; p.mu_lat = 1.0; }
    c[vdsim::WHEEL_RL].mu_long = 0.2;
    c[vdsim::WHEEL_RL].mu_lat  = 0.2;
    return c;
}

vdsim::State init_state(double vx, double R) {
    vdsim::State s; s.velocity.x() = vx;
    const double w = (R > 0.0) ? vx / R : 0.0;
    s.wheel_spin = {{w, w, w, w}};
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr,
            "usage: %s <vehicle.yaml> <tire.yaml> <Open|Locked|LSD> <out.csv>\n",
            argv[0]);
        return 2;
    }
    auto vp = vdsim::VehicleParams::from_yaml(argv[1]);
    const auto tp = vdsim::TireParams::from_yaml(argv[2]);
    const std::string diff = argv[3];
    if (diff == "Open")   vp.differential = vdsim::VehicleParams::Differential::Open;
    else if (diff == "Locked") vp.differential = vdsim::VehicleParams::Differential::Locked;
    else if (diff == "LSD")    vp.differential = vdsim::VehicleParams::Differential::LSD;
    else { std::fprintf(stderr, "unknown diff: %s\n", diff.c_str()); return 2; }

    auto dyn = vdsim::create_seven_dof();
    const vdsim::SolverParams sp;
    dyn->initialize(vp, tp, sp);
    dyn->reset(init_state(2.0, vp.wheel_radius_nominal));

    vdsim::CmdL4 cmd; cmd.throttle = 1.0;
    const vdsim::ControlInput u = cmd;
    const auto contacts = split_mu_contacts();

    std::ofstream csv(argv[4]);
    csv << "t,vx,omega_RL,omega_RR,Fz_RL,Fz_RR\n";

    const double T = 4.0, dt = 0.005;
    const int N = static_cast<int>(std::round(T / dt));
    for (int i = 0; i <= N; ++i) {
        const auto& s = dyn->state();
        const auto Fz = dyn->tire_Fz();
        csv << (i*dt) << ',' << s.velocity.x() << ','
            << s.wheel_spin[vdsim::WHEEL_RL] << ','
            << s.wheel_spin[vdsim::WHEEL_RR] << ','
            << Fz[vdsim::WHEEL_RL] << ',' << Fz[vdsim::WHEEL_RR] << '\n';
        if (i < N) dyn->step(u, contacts, dt);
    }
    std::fprintf(stderr,
        "[split_mu_demo] %s: vx_end=%.3f  omega_RL=%.2f  omega_RR=%.2f\n",
        diff.c_str(), dyn->state().velocity.x(),
        dyn->state().wheel_spin[vdsim::WHEEL_RL],
        dyn->state().wheel_spin[vdsim::WHEEL_RR]);
    return 0;
}
