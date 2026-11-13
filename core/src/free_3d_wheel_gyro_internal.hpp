// L5 wheel-spin gyroscopic term (work order 50). Internal to the core:
// free_3d_dynamics.cpp uses it, the unit tests check it directly. Header-only so the
// dynamics keeps the 0.8.1 arithmetic of the legacy term bit for bit.
#pragma once

#include "vdsim/params.hpp"
#include "vdsim/types.hpp"

#include <array>
#include <cmath>

namespace vdsim::detail {

/**
 * @brief Unit spin axis of one wheel in body axes, oriented toward body +y.
 *
 * Small-angle steer tilt as in 0.8.1: (-delta, 1, 0), normalised. WheelGyro::Full also
 * tilts the axis by the camber: z = -tan(gamma), so at zero steer the axis is
 * (0, cos gamma, -sin gamma). gamma is the vehicle-frame camber of the corner model
 * (+ = wheel top toward body +y, as free_3d_wheel_camber reports it). A zero camber
 * leaves the legacy vector untouched.
 *
 * @param mode  Legacy or Full (Off never asks for an axis).
 * @param delta Steer + DAE toe of the wheel [rad].
 * @param gamma Vehicle-frame camber [rad]; ignored unless mode is Full.
 * @return Unit spin axis (body axes).
 */
inline Vec3 wheel_spin_axis(WheelGyro mode, double delta, double gamma) noexcept {
    Vec3 s_hat(-delta, 1.0, 0.0);
    if (mode == WheelGyro::Full && gamma != 0.0) s_hat.z() = -std::tan(gamma);
    s_hat.normalize();
    return s_hat;
}

/**
 * @brief Gyroscopic moment of the four spinning wheels on the body, -omega x H.
 *
 * H = sum_i I_i * omega_i * s_hat_i (wheel_spin_axis). The caller adds the result to
 * the body torque; Legacy reproduces the 0.8.1 "tau -= omega x H" bit for bit.
 *
 * @param mode   Legacy, Full or Off (Off returns zero).
 * @param omega  Body angular velocity (body axes) [rad/s].
 * @param I      Wheel spin inertia [kg m^2].
 * @param spin   Wheel spin rate [rad/s].
 * @param delta  Steer + DAE toe per wheel [rad].
 * @param gamma  Vehicle-frame camber per wheel [rad].
 * @return Moment on the body (body axes) [N m].
 */
inline Vec3 wheel_gyro_moment(WheelGyro mode, const Vec3& omega,
                              const std::array<double, NUM_WHEELS>& I,
                              const std::array<double, NUM_WHEELS>& spin,
                              const std::array<double, NUM_WHEELS>& delta,
                              const std::array<double, NUM_WHEELS>& gamma) noexcept {
    if (mode == WheelGyro::Off) return Vec3::Zero();
    Vec3 H_spin = Vec3::Zero();
    for (int i = 0; i < NUM_WHEELS; ++i)
        H_spin += (I[i] * spin[i]) * wheel_spin_axis(mode, delta[i], gamma[i]);
    return -omega.cross(H_spin);
}

}  // namespace vdsim::detail
