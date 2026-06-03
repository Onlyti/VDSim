// Native (no-precompute) double-wishbone kinematics solver.
// Mirrors tools/kinematics/dw_3d_solver.py's algorithm in C++.

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
using Eigen::Vector3d;

Vector3d yaml_vec(const YAML::Node& n) {
    return Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

Matrix3d rodrigues(const Vector3d& axis, double theta) {
    Matrix3d K;
    K <<           0, -axis(2),  axis(1),
              axis(2),       0, -axis(0),
             -axis(1),  axis(0),       0;
    return Matrix3d::Identity() + std::sin(theta) * K
                                + (1.0 - std::cos(theta)) * (K * K);
}

// Sphere intersection of three: find P with |P-c_i| = r_i for i=1,2,3.
// Picks the solution closer to `near`.  Returns false if no real intersection.
bool trilaterate3(const Vector3d& c1, double r1,
                   const Vector3d& c2, double r2,
                   const Vector3d& c3, double r3,
                   const Vector3d& near, Vector3d& out) {
    Vector3d ex = c2 - c1;
    double d = ex.norm();
    if (d < 1e-9) return false;
    ex /= d;
    Vector3d tmp = c3 - c1;
    double i_ = ex.dot(tmp);
    Vector3d ey_raw = tmp - i_ * ex;
    double ny = ey_raw.norm();
    if (ny < 1e-9) return false;
    Vector3d ey = ey_raw / ny;
    Vector3d ez = ex.cross(ey);
    double j_ = ey.dot(c3 - c1);
    double x = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    double y = (r1 * r1 - r3 * r3 + i_ * i_ + j_ * j_ - 2.0 * i_ * x) / (2.0 * j_);
    double z_sq = r1 * r1 - x * x - y * y;
    if (z_sq < -1e-9) return false;
    double z = std::sqrt(std::max(0.0, z_sq));
    Vector3d p_pos = c1 + x * ex + y * ey + z * ez;
    Vector3d p_neg = c1 + x * ex + y * ey - z * ez;
    out = ((p_pos - near).norm() <= (p_neg - near).norm()) ? p_pos : p_neg;
    return true;
}

class DWNativeKinematics final : public ISuspensionKinematics {
public:
    explicit DWNativeKinematics(const std::string& yaml_path) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        const std::string type = root["type"]
            ? root["type"].as<std::string>() : "";
        if (type != "double_wishbone") {
            throw std::runtime_error(
                "DWNative: expected a 3D hardpoint config with type=double_wishbone "
                "(got '" + type + "'). Configs keyed 'topology:' are 2D-legacy "
                "(side-view analyzer); use a 'type:'-schema config such as "
                "dw_front_sports.yaml.");
        }
        side_ = root["side"] ? root["side"].as<std::string>() : "left";

        wheel_static_    = yaml_vec(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec(root["wheel"]["spin_axis"]);

        lca_cf_ = yaml_vec(root["lca"]["chassis_front"]);
        lca_cr_ = yaml_vec(root["lca"]["chassis_rear"]);
        lca_axis_ = (lca_cr_ - lca_cf_).normalized();
        lca_pivot_ = lca_cf_;
        lca_knuckle_static_ = yaml_vec(root["lca"]["knuckle"]);

        uca_cf_ = yaml_vec(root["uca"]["chassis_front"]);
        uca_cr_ = yaml_vec(root["uca"]["chassis_rear"]);
        uca_axis_ = (uca_cr_ - uca_cf_).normalized();
        uca_pivot_ = uca_cf_;
        uca_knuckle_static_ = yaml_vec(root["uca"]["knuckle"]);

        L_LU_ = (uca_knuckle_static_ - lca_knuckle_static_).norm();
        tr_inner_static_   = yaml_vec(root["tie_rod"]["rack"]);
        tr_knuckle_static_ = yaml_vec(root["tie_rod"]["knuckle"]);
        L_tr_ = (tr_knuckle_static_ - tr_inner_static_).norm();
        L_LT_ = (tr_knuckle_static_ - lca_knuckle_static_).norm();
        L_UT_ = (tr_knuckle_static_ - uca_knuckle_static_).norm();

        // Static knuckle frame (ax = kingpin, ay in TK plane perp, az = cross)
        Vector3d ax0 = (uca_knuckle_static_ - lca_knuckle_static_).normalized();
        Vector3d tk_off0 = tr_knuckle_static_ - lca_knuckle_static_;
        Vector3d ay0_raw = tk_off0 - ax0.dot(tk_off0) * ax0;
        Vector3d ay0 = ay0_raw / std::max(1e-9, ay0_raw.norm());
        Vector3d az0 = ax0.cross(ay0);
        R0_.col(0) = ax0; R0_.col(1) = ay0; R0_.col(2) = az0;
        wheel_off_local_ = R0_.transpose() * (wheel_static_ - lca_knuckle_static_);
    }

    Output compute(double wheel_travel,
                   double steer_input) const noexcept override {
        const double target_z = wheel_static_(2) + wheel_travel;
        // Newton on θ_lca s.t. TRUE wheel z (after full solve) = target.
        double theta_l = 0.0;
        for (int it = 0; it < 30; ++it) {
            KinState s; if (!kinematic_at(theta_l, steer_input, s)) break;
            const double err = s.wheel_pos(2) - target_z;
            if (std::abs(err) < 1e-7) break;
            const double dth = 1e-5;
            KinState sp; if (!kinematic_at(theta_l + dth, steer_input, sp)) break;
            const double slope = (sp.wheel_pos(2) - s.wheel_pos(2)) / dth;
            if (std::abs(slope) < 1e-9) break;
            theta_l -= err / slope;
        }
        KinState s;
        if (!kinematic_at(theta_l, steer_input, s)) return Output{};

        Output o;
        const Vector3d& spin = s.spin_axis_world;
        o.camber = std::atan2(-spin(2), std::abs(spin(1)));
        if (side_ == "right") o.camber = -o.camber;
        o.toe = std::atan2(spin(0), spin(1));
        if (side_ == "right") o.toe = -o.toe;
        o.track_change = s.wheel_pos(1) - wheel_static_(1);
        if (side_ == "right") o.track_change = -o.track_change;
        const Vector3d kp = s.uk - s.lk;
        const double kp_xz = std::hypot(kp(0), kp(2));
        o.caster = (kp_xz > 1e-9) ? std::atan2(kp(0), kp(2)) : 0.0;
        return o;
    }

private:
    struct KinState {
        Vector3d lk, uk, tk;
        Vector3d wheel_pos;
        Vector3d spin_axis_world;
        Matrix3d R_now;
    };

    Vector3d lca_knuckle_at(double theta) const {
        return lca_pivot_ + rodrigues(lca_axis_, theta)
                              * (lca_knuckle_static_ - lca_pivot_);
    }
    Vector3d uca_knuckle_at(double theta) const {
        return uca_pivot_ + rodrigues(uca_axis_, theta)
                              * (uca_knuckle_static_ - uca_pivot_);
    }

    bool kinematic_at(double theta_l, double steer_input,
                       KinState& out) const {
        out.lk = lca_knuckle_at(theta_l);
        // Solve UCA θ_u so |UK - LK| = L_LU
        double theta_u = 0.0;
        for (int it = 0; it < 30; ++it) {
            Vector3d uk = uca_knuckle_at(theta_u);
            const double d = (uk - out.lk).norm();
            const double err = d - L_LU_;
            if (std::abs(err) < 1e-7) { out.uk = uk; goto have_uca; }
            const double dth = 1e-5;
            Vector3d uk_p = uca_knuckle_at(theta_u + dth);
            const double slope = ((uk_p - out.lk).norm() - d) / dth;
            if (std::abs(slope) < 1e-9) break;
            theta_u -= err / slope;
        }
        out.uk = uca_knuckle_at(theta_u);
have_uca:;
        const Vector3d tr_inner = tr_inner_static_
                                  + Vector3d(0.0, steer_input, 0.0);
        if (!trilaterate3(out.lk, L_LT_, out.uk, L_UT_, tr_inner, L_tr_,
                          tr_knuckle_static_, out.tk)) return false;

        const Vector3d ax = (out.uk - out.lk).normalized();
        const Vector3d tk_off = out.tk - out.lk;
        const Vector3d ay_raw = tk_off - ax.dot(tk_off) * ax;
        const Vector3d ay = ay_raw / std::max(1e-9, ay_raw.norm());
        const Vector3d az = ax.cross(ay);
        out.R_now.col(0) = ax; out.R_now.col(1) = ay; out.R_now.col(2) = az;
        out.wheel_pos = out.lk + out.R_now * wheel_off_local_;
        const Matrix3d R_delta = out.R_now * R0_.transpose();
        out.spin_axis_world = R_delta * wheel_spin_axis_;
        return true;
    }

    std::string side_;
    Vector3d wheel_static_, wheel_spin_axis_;
    Vector3d lca_cf_, lca_cr_, lca_axis_, lca_pivot_, lca_knuckle_static_;
    Vector3d uca_cf_, uca_cr_, uca_axis_, uca_pivot_, uca_knuckle_static_;
    Vector3d tr_inner_static_, tr_knuckle_static_;
    double L_LU_ {0}, L_LT_ {0}, L_UT_ {0}, L_tr_ {0};
    Matrix3d R0_;
    Vector3d wheel_off_local_;
};

}  // namespace

std::unique_ptr<ISuspensionKinematics>
create_dw_native_kinematics(const std::string& yaml_path) {
    return std::make_unique<DWNativeKinematics>(yaml_path);
}

}  // namespace vdsim
