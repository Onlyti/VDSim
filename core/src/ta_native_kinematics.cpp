// Native trailing-arm kinematics — single revolute joint.
// Newton on θ_arm s.t. true wheel z = static + travel.

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

class TANativeKinematics final : public ISuspensionKinematics {
public:
    explicit TANativeKinematics(const std::string& yaml_path) {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (root["type"].as<std::string>() != "trailing_arm")
            throw std::runtime_error("TANative: expected type=trailing_arm");
        side_ = root["side"] ? root["side"].as<std::string>() : "left";
        wheel_static_    = yaml_vec(root["wheel"]["center"]);
        wheel_spin_axis_ = yaml_vec(root["wheel"]["spin_axis"]);
        pivot_in_  = yaml_vec(root["arm_pivot"]["chassis_inboard"]);
        pivot_out_ = yaml_vec(root["arm_pivot"]["chassis_outboard"]);
        arm_axis_  = (pivot_out_ - pivot_in_).normalized();
        pivot_     = pivot_in_;
        wheel_off_ = wheel_static_ - pivot_;
    }

    Output compute(double wheel_travel,
                   double /*steer_input*/) const noexcept override {
        const double target_z = wheel_static_(2) + wheel_travel;
        double theta = 0.0;
        for (int it = 0; it < 30; ++it) {
            const Vector3d w = pivot_ + rodrigues(arm_axis_, theta) * wheel_off_;
            const double err = w(2) - target_z;
            if (std::abs(err) < 1e-7) break;
            const double dth = 1e-5;
            const Vector3d w_p = pivot_ + rodrigues(arm_axis_, theta + dth) * wheel_off_;
            const double slope = (w_p(2) - w(2)) / dth;
            if (std::abs(slope) < 1e-9) break;
            theta -= err / slope;
        }
        const Matrix3d R = rodrigues(arm_axis_, theta);
        const Vector3d wheel = pivot_ + R * wheel_off_;
        const Vector3d spin  = R * wheel_spin_axis_;

        Output o;
        o.camber = std::atan2(-spin(2), std::abs(spin(1)));
        if (side_ == "right") o.camber = -o.camber;
        o.toe = std::atan2(spin(0), spin(1));
        if (side_ == "right") o.toe = -o.toe;
        o.track_change = wheel(1) - wheel_static_(1);
        if (side_ == "right") o.track_change = -o.track_change;
        o.caster = 0.0;     // no defined kingpin for trailing arm
        return o;
    }

private:
    std::string side_;
    Vector3d wheel_static_, wheel_spin_axis_;
    Vector3d pivot_in_, pivot_out_, arm_axis_, pivot_, wheel_off_;
};

}  // namespace

std::unique_ptr<ISuspensionKinematics>
create_ta_native_kinematics(const std::string& yaml_path) {
    return std::make_unique<TANativeKinematics>(yaml_path);
}

}  // namespace vdsim
