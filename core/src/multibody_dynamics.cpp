#include "vdsim/multibody.hpp"

#include <algorithm>
#include <cmath>

namespace vdsim::mb {

namespace {

void integrate_axis(double& q, double& qd, double m, double c, double k,
                    double tau, double h) {
    const double acc = (tau - c * qd - k * q) / m;
    qd += h * acc;
    q += h * qd;
}

}  // namespace

void step_corner_dynamics(CornerDynamicsState& state,
                          SuspensionTopology& topo,
                          const WheelLoad& load,
                          double dt) {
    if (dt <= 0.0) return;
    ensure_default_bushings(topo);
    double q_eq_toe = 0.0, q_eq_camber = 0.0;
    compliance_targets_rad(topo, load, q_eq_toe, q_eq_camber);
    const CornerDaeParams p = corner_dae_params(topo);
    const double tau_toe = p.k_toe * q_eq_toe;
    const double tau_camber = p.k_camber * q_eq_camber;
    const int nsub =
        std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
    const double h = dt / static_cast<double>(nsub);
    for (int i = 0; i < nsub; ++i) {
        integrate_axis(state.q_toe_rad, state.q_toe_dot, p.m_toe, p.c_toe, p.k_toe,
                       tau_toe, h);
        integrate_axis(state.q_camber_rad, state.q_camber_dot, p.m_camber, p.c_camber,
                       p.k_camber, tau_camber, h);
    }
    topo.compliance_toe_deg    = state.q_toe_rad * 180.0 / M_PI;
    topo.compliance_camber_deg = state.q_camber_rad * 180.0 / M_PI;
}

}  // namespace vdsim::mb
