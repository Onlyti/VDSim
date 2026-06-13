#include "vdsim/multibody_math.hpp"

#include "vdsim/multibody.hpp"

#include <algorithm>
#include <cmath>

namespace vdsim::mb {

Mat3 rodrigues(const Vec3& axis_unit, double theta) {
    Mat3 K;
    K << 0, -axis_unit.z(), axis_unit.y(),
         axis_unit.z(), 0, -axis_unit.x(),
         -axis_unit.y(), axis_unit.x(), 0;
    return Mat3::Identity() + std::sin(theta) * K
                              + (1.0 - std::cos(theta)) * (K * K);
}

Mat3 axis_angle_to_R(const Vec3& v) {
    const double a = v.norm();
    if (a < 1e-12) return Mat3::Identity();
    return rodrigues(v / a, a);
}

double corner_inertia_about_axis(const std::vector<RigidBody>& bodies,
                                 const Vec3& axis, const Vec3& pivot,
                                 double floor) {
    const Vec3 axis_u = axis.normalized();
    double I = 0.0;
    for (const auto& b : bodies) {
        if (b.id == "chassis" || b.mass <= 0.0) continue;
        const Vec3 r = b.cg_local - pivot;
        const Vec3 r_perp = r - axis_u * axis_u.dot(r);
        I += b.mass * r_perp.squaredNorm();
    }
    return std::max(floor, I);
}

LumpedCorner lump_corner_about_axis(const std::vector<RigidBody>& bodies,
                                    const Vec3& axis, const Vec3& pivot) {
    const Vec3 axis_u = axis.normalized();
    double mass = 0.0;
    Vec3 mass_pos = Vec3::Zero();
    for (const auto& b : bodies) {
        if (b.id == "chassis" || b.mass <= 0.0) continue;
        mass += b.mass;
        mass_pos += b.mass * b.cg_local;
    }
    if (mass < 1e-6) mass = 9.0;
    LumpedCorner out;
    out.mass = mass;
    out.com = mass_pos / mass;
    const double i_pivot = corner_inertia_about_axis(bodies, axis_u, pivot, 0.2);
    const Vec3 com_rel = out.com - pivot;
    const Vec3 com_perp = com_rel - axis_u * axis_u.dot(com_rel);
    out.i_com = std::max(0.05, i_pivot - mass * com_perp.squaredNorm());
    return out;
}

}  // namespace vdsim::mb
