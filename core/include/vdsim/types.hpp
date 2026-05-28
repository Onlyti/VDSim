#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vdsim {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Quat = Eigen::Quaterniond;  // body -> world (Eigen convention)

constexpr int WHEEL_FL  = 0;
constexpr int WHEEL_FR  = 1;
constexpr int WHEEL_RL  = 2;
constexpr int WHEEL_RR  = 3;
constexpr int NUM_WHEELS = 4;

}  // namespace vdsim
