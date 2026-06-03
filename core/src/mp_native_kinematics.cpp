// Native MacPherson kinematics — cylindrical-joint constraint.
// Outer Newton on θ_lca for true wheel z + inner LM on knuckle rotation R.

#include "vdsim/suspension.hpp"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace vdsim {

namespace {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

Vector3d yaml_vec(const YAML::Node& n) {
    return Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

Matrix3d rodrigues(const Vector3d& axis_unit, double theta) {
    Matrix3d K;
    K <<           0, -axis_unit(2),  axis_unit(1),
              axis_unit(2),       0, -axis_unit(0),
             -axis_unit(1),  axis_unit(0),       0;
    return Matrix3d::Identity() + std::sin(theta) * K
                                + (1.0 - std::cos(theta)) * (K * K);
}

Matrix3d axis_angle_to_R(const Vector3d& v) {
    const double a = v.norm();
    if (a < 1e-12) return Matrix3d::Identity();
    return rodrigues(v / a, a);
}

// Compact Levenberg–Marquardt for small dense problems.  Numerical Jacobian.
// f: (VectorXd in) -> VectorXd of residuals.
template <typename F>
VectorXd lm_solve(F&& f, VectorXd x0,
                  int max_iter = 100, double tol = 1e-9) {
    VectorXd x = x0;
    double lambda = 1e-3;
    VectorXd r = f(x);
    double cost = r.squaredNorm();
    const int n = x.size();
    const int m = r.size();
    for (int it = 0; it < max_iter; ++it) {
        // Numerical Jacobian (forward difference)
        MatrixXd J(m, n);
        const double h = 1e-6;
        for (int i = 0; i < n; ++i) {
            VectorXd xp = x; xp(i) += h;
            VectorXd rp = f(xp);
            J.col(i) = (rp - r) / h;
        }
        MatrixXd JTJ = J.transpose() * J;
        VectorXd JTr = J.transpose() * r;
        MatrixXd A = JTJ;
        A.diagonal().array() += lambda * JTJ.diagonal().array().max(1e-12);
        VectorXd dx = A.ldlt().solve(-JTr);
        VectorXd x_new = x + dx;
        VectorXd r_new = f(x_new);
        const double cost_new = r_new.squaredNorm();
        if (cost_new < cost) {
            x = x_new; r = r_new; cost = cost_new;
            lambda *= 0.5;
            if (dx.norm() < tol) break;
        } else {
            lambda *= 2.0;
            if (lambda > 1e10) break;
        }
    }
    return x;
}

class MPNativeKinematics final : public ISuspensionKinematics {
public:
    explicit MPNativeKinematics(const std::string& yaml_path) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        const std::string type = root["type"] ? root["type"].as<std::string>() : "";
        if (type != "macpherson")
            throw std::runtime_error(
                "MPNative: expected a 3D hardpoint config with type=macpherson (got '"
                + type + "'). Configs keyed 'topology:' are 2D-legacy (side-view "
                "analyzer); use a 'type:'-schema config such as mp_front_sedan.yaml.");
        side_ = root["side"] ? root["side"].as<std::string>() : "left";
        wheel_static_    = yaml_vec(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec(root["wheel"]["spin_axis"]);

        lca_cf_ = yaml_vec(root["lca"]["chassis_front"]);
        lca_cr_ = yaml_vec(root["lca"]["chassis_rear"]);
        lca_axis_ = (lca_cr_ - lca_cf_).normalized();
        lca_pivot_ = lca_cf_;
        lca_knuckle_static_ = yaml_vec(root["lca"]["knuckle"]);

        strut_top_     = yaml_vec(root["strut"]["top"]);
        strut_bottom_  = yaml_vec(root["strut"]["bottom"]);
        tube_axis_body_ = (strut_top_ - strut_bottom_).normalized();
        L_tr_inner_static_  = yaml_vec(root["tie_rod"]["rack"]);
        tr_knuckle_static_  = yaml_vec(root["tie_rod"]["knuckle"]);
        L_tr_ = (tr_knuckle_static_ - L_tr_inner_static_).norm();

        off_SK_     = strut_bottom_     - lca_knuckle_static_;
        off_TK_     = tr_knuckle_static_ - lca_knuckle_static_;
        off_wheel_  = wheel_static_     - lca_knuckle_static_;
    }

    Output compute(double wheel_travel,
                   double steer_input) const noexcept override {
        const double target_z = wheel_static_(2) + wheel_travel;
        Vector3d aa = Vector3d::Zero();    // initial axis-angle
        double theta_l = 0.0;
        // Outer Newton on θ_lca for true wheel z (each iteration runs LM).
        for (int it = 0; it < 30; ++it) {
            const auto st = solve_R_at(theta_l, steer_input, aa);
            if (!st.valid) break;
            const double err = st.wheel(2) - target_z;
            if (std::abs(err) < 1e-7) { aa = st.aa; break; }
            const double dth = 1e-5;
            const auto st_p = solve_R_at(theta_l + dth, steer_input, st.aa);
            if (!st_p.valid) break;
            const double slope = (st_p.wheel(2) - st.wheel(2)) / dth;
            if (std::abs(slope) < 1e-9) break;
            aa = st.aa;
            theta_l -= err / slope;
        }
        const auto st = solve_R_at(theta_l, steer_input, aa);
        if (!st.valid) return Output{};

        const Vector3d& spin = st.spin;
        Output o;
        o.camber = std::atan2(-spin(2), std::abs(spin(1)));
        if (side_ == "right") o.camber = -o.camber;
        o.toe = std::atan2(spin(0), spin(1));
        if (side_ == "right") o.toe = -o.toe;
        o.track_change = st.wheel(1) - wheel_static_(1);
        if (side_ == "right") o.track_change = -o.track_change;
        const Vector3d kp = st.sk - strut_top_;
        const double kp_xz = std::hypot(kp(0), kp(2));
        o.caster = (kp_xz > 1e-9) ? std::atan2(kp(0), -kp(2)) : 0.0;
        return o;
    }

private:
    struct State { bool valid {false}; Vector3d wheel, sk, spin, aa; };

    Vector3d lca_knuckle_at(double theta) const {
        return lca_pivot_ + rodrigues(lca_axis_, theta)
                              * (lca_knuckle_static_ - lca_pivot_);
    }

    State solve_R_at(double theta_l, double steer_input,
                      Vector3d x0_aa) const {
        State s;
        const Vector3d lk = lca_knuckle_at(theta_l);
        const Vector3d tr_inner = L_tr_inner_static_
                                  + Vector3d(0.0, steer_input, 0.0);
        // Residuals: 3 cross(SK - ST, R @ tube_axis_body) + 1 (|TK - TR_inner| - L_tr)
        auto residuals = [&](const VectorXd& v) -> VectorXd {
            Vector3d aa = v;
            Matrix3d R = axis_angle_to_R(aa);
            Vector3d sk = lk + R * off_SK_;
            Vector3d tk = lk + R * off_TK_;
            Vector3d tube_world = R * tube_axis_body_;
            Vector3d cross = (sk - strut_top_).cross(tube_world);
            VectorXd r(4);
            r(0) = cross(0); r(1) = cross(1); r(2) = cross(2);
            r(3) = (tk - tr_inner).norm() - L_tr_;
            return r;
        };
        VectorXd aa_init(3); aa_init << x0_aa(0), x0_aa(1), x0_aa(2);
        VectorXd aa_solved = lm_solve(residuals, aa_init, 100, 1e-9);
        Vector3d aa(aa_solved(0), aa_solved(1), aa_solved(2));
        Matrix3d R = axis_angle_to_R(aa);
        s.aa    = aa;
        s.sk    = lk + R * off_SK_;
        s.wheel = lk + R * off_wheel_;
        s.spin  = R * wheel_spin_axis_;
        s.valid = true;
        return s;
    }

    std::string side_;
    Vector3d wheel_static_, wheel_spin_axis_;
    Vector3d lca_cf_, lca_cr_, lca_axis_, lca_pivot_, lca_knuckle_static_;
    Vector3d strut_top_, strut_bottom_, tube_axis_body_;
    Vector3d L_tr_inner_static_, tr_knuckle_static_;
    double L_tr_ {0};
    Vector3d off_SK_, off_TK_, off_wheel_;
};

}  // namespace

std::unique_ptr<ISuspensionKinematics>
create_mp_native_kinematics(const std::string& yaml_path) {
    return std::make_unique<MPNativeKinematics>(yaml_path);
}

}  // namespace vdsim
