#include "free_3d_reaction_internal.hpp"

#include <Eigen/Geometry>

namespace vdsim::detail {

std::optional<double> knuckle_pitch_rate(const mb::IHardJointDaeModel& dae,
                                         double travel_z, double steer_rack_dy,
                                         const Vec3& seed, double dz) {
    const mb::WheelPose pp = dae.pose_at_travel(travel_z + dz, steer_rack_dy, seed);
    const mb::WheelPose pm = dae.pose_at_travel(travel_z - dz, steer_rack_dy, seed);
    const Quat ident = Quat::Identity();
    if (pp.orientation.coeffs() == ident.coeffs() && pm.orientation.coeffs() == ident.coeffs())
        return std::nullopt;
    const Quat dq = (pp.orientation * pm.orientation.conjugate()).normalized();
    const Eigen::AngleAxisd aa(dq);
    return (aa.angle() * aa.axis()).y() / (2.0 * dz);
}

KnuckleReactionSplit split_knuckle_reaction(double M_pitch, double theta_p,
                                            const Vec3& t_body, const Vec3& r_wheel,
                                            const Vec3& r_mount) {
    KnuckleReactionSplit out;
    const double tt = t_body.squaredNorm();
    if (!(tt > 1e-12)) return out;
    out.force_on_wheel = (M_pitch * theta_p / tt) * t_body;
    out.force_on_body  = -out.force_on_wheel;
    out.couple_on_body = Vec3(0.0, M_pitch, 0.0) - (r_wheel - r_mount).cross(out.force_on_wheel);
    return out;
}

}  // namespace vdsim::detail
