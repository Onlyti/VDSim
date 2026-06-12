#pragma once

// Shared multibody math helpers (Ld4 v0.6 M1).
//
// Previously duplicated between multibody_hard_dae.cpp (corner DAE, runtime
// L4 step) and multibody_featherstone.cpp (lumped revolute tree, offline).
// Keeping the inertia-lump and rotation math in one place removes the
// "keep in sync" hazard flagged in design/LD4_MULTIBODY.md §7.

#include <vector>

#include <Eigen/Core>

#include "vdsim/types.hpp"

namespace vdsim::mb {

struct RigidBody;  // defined in multibody.hpp

// Rodrigues rotation about a unit axis by theta [rad].
Mat3 rodrigues(const Vec3& axis_unit, double theta);

// Axis-angle vector (magnitude = angle, direction = axis) -> rotation matrix.
// Returns identity for a near-zero vector.
Mat3 axis_angle_to_R(const Vec3& v);

// Moment of inertia of the unsprung corner bodies about a pivot axis.
// Each body CG offset from `pivot` is projected onto the plane normal to
// `axis`; "chassis" and massless bodies are skipped. `axis` need not be unit
// (normalized internally). Floored at `floor` to keep the corner DAE well-posed.
double corner_inertia_about_axis(const std::vector<RigidBody>& bodies,
                                 const Vec3& axis, const Vec3& pivot,
                                 double floor = 0.2);

// Single-rigid-body reduction of the corner about a pivot axis: total unsprung
// mass, its CG, and the inertia about the CG (parallel-axis from the pivot).
struct LumpedCorner {
    double mass {0.0};
    Vec3   com {Vec3::Zero()};   // topology frame
    double i_com {0.0};          // about an axis through the CG parallel to `axis`
};

LumpedCorner lump_corner_about_axis(const std::vector<RigidBody>& bodies,
                                    const Vec3& axis, const Vec3& pivot);

}  // namespace vdsim::mb
