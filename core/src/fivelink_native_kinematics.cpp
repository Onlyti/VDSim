// Native 5-link rear kinematics — 6-DOF knuckle pose + 5 link constraints.
// Uses a compact Levenberg–Marquardt with numerical Jacobian.

#include "vdsim/suspension.hpp"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <array>
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

template <typename F>
VectorXd lm_solve(F&& f, VectorXd x0,
                  int max_iter = 200, double tol = 1e-9) {
    VectorXd x = x0;
    double lambda = 1e-3;
    VectorXd r = f(x);
    double cost = r.squaredNorm();
    const int n = x.size();
    const int m = r.size();
    for (int it = 0; it < max_iter; ++it) {
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

class FiveLinkNativeKinematics final : public ISuspensionKinematics {
public:
    explicit FiveLinkNativeKinematics(const std::string& yaml_path) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        const std::string type = root["type"] ? root["type"].as<std::string>() : "";
        if (type != "five_link")
            throw std::runtime_error(
                "5LinkNative: expected a 3D hardpoint config with type=five_link (got '"
                + type + "'). Configs keyed 'topology:' are 2D-legacy (side-view "
                "analyzer); use a 'type:'-schema config such as 5link_rear_sports.yaml.");
        side_ = root["side"] ? root["side"].as<std::string>() : "left";
        wheel_static_    = yaml_vec(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec(root["wheel"]["spin_axis"]);

        static constexpr const char* LINK_NAMES[5] = {
            "upper_fore", "upper_aft", "lower_fore", "lower_aft", "toe_link",
        };
        const YAML::Node& links = root["links"];
        for (int i = 0; i < 5; ++i) {
            const auto& L = links[LINK_NAMES[i]];
            chassis_pts_[i] = yaml_vec(L["chassis"]);
            knuckle_pts_static_[i] = yaml_vec(L["knuckle"]);
            link_len_[i] = (knuckle_pts_static_[i] - chassis_pts_[i]).norm();
            knuckle_off_[i] = knuckle_pts_static_[i] - wheel_static_;
        }
    }

    Output compute(double wheel_travel,
                   double /*steer_input*/) const noexcept override {
        const double target_z = wheel_static_(2) + wheel_travel;

        // Pose = [x, y, z, axis-angle (3)] → 6 DOF.
        VectorXd pose(6);
        pose << wheel_static_(0), wheel_static_(1), wheel_static_(2) + wheel_travel,
                 0.0, 0.0, 0.0;

        auto residuals = [&](const VectorXd& v) -> VectorXd {
            Vector3d pos(v(0), v(1), v(2));
            Vector3d aa(v(3), v(4), v(5));
            Matrix3d R = axis_angle_to_R(aa);
            VectorXd r(6);
            for (int i = 0; i < 5; ++i) {
                Vector3d k = pos + R * knuckle_off_[i];
                r(i) = (k - chassis_pts_[i]).norm() - link_len_[i];
            }
            r(5) = pos(2) - target_z;
            return r;
        };

        VectorXd solved = lm_solve(residuals, pose, 200, 1e-9);
        Vector3d pos(solved(0), solved(1), solved(2));
        Vector3d aa(solved(3), solved(4), solved(5));
        Matrix3d R = axis_angle_to_R(aa);
        Vector3d spin = R * wheel_spin_axis_;

        Output o;
        o.camber = std::atan2(-spin(2), std::abs(spin(1)));
        if (side_ == "right") o.camber = -o.camber;
        o.toe = std::atan2(spin(0), spin(1));
        if (side_ == "right") o.toe = -o.toe;
        o.track_change = pos(1) - wheel_static_(1);
        if (side_ == "right") o.track_change = -o.track_change;
        o.caster = 0.0;
        return o;
    }

private:
    std::string side_;
    Vector3d wheel_static_, wheel_spin_axis_;
    std::array<Vector3d, 5> chassis_pts_, knuckle_pts_static_, knuckle_off_;
    std::array<double, 5> link_len_;
};

}  // namespace

std::unique_ptr<ISuspensionKinematics>
create_5link_native_kinematics(const std::string& yaml_path) {
    return std::make_unique<FiveLinkNativeKinematics>(yaml_path);
}

}  // namespace vdsim
