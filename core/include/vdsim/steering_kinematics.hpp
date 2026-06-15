#pragma once

// ISteeringKinematics — converts rack_travel [m] to per-wheel steer angles.
// Analogous to ISuspensionKinematics (rack_travel → FL/FR toe, not suspension deflection).
//
// Pluggable per dynamics level:
//   RatioSteeringKinematics : simple ratio (Ld1/2)
//   AckermannKinematics     : Ackermann geometry, FL ≠ FR (Ld3)
//   HardpointSteeringKin    : lookup table from Ld4 hardpoint data

#include <memory>

namespace vdsim {

struct ISteeringKinematics {
    struct Output {
        double angle_fl {0.0};   // [rad]  front-left wheel steer angle
        double angle_fr {0.0};   // [rad]  front-right wheel steer angle
        double angle_avg() const { return 0.5 * (angle_fl + angle_fr); }
    };

    virtual Output compute(double rack_travel) const = 0;
    virtual ~ISteeringKinematics() = default;
};

// ─── RatioSteeringKinematics ───────────────────────────────────────────────
// δ = rack_travel / steering_ratio  (both wheels identical — Ld1/2 bicycle)
class RatioSteeringKinematics final : public ISteeringKinematics {
public:
    explicit RatioSteeringKinematics(double steering_ratio)
        : ratio_(steering_ratio > 0.0 ? steering_ratio : 1.0) {}

    Output compute(double rack_travel) const override {
        const double angle = rack_travel / ratio_;
        return {angle, angle};
    }

private:
    double ratio_;
};

// ─── AckermannKinematics ───────────────────────────────────────────────────
// Ackermann geometry: inner wheel turns more than outer.
// δ_inner = atan(L / (R - t/2)),  δ_outer = atan(L / (R + t/2))
// Linearised for small angles: δ_FL ≠ δ_FR from rack_travel via separate ratios.
class AckermannKinematics final : public ISteeringKinematics {
public:
    // ratio_fl, ratio_fr: rack_travel [m] → wheel angle [rad] per side
    AckermannKinematics(double ratio_fl, double ratio_fr)
        : ratio_fl_(ratio_fl > 0.0 ? ratio_fl : 1.0)
        , ratio_fr_(ratio_fr > 0.0 ? ratio_fr : 1.0) {}

    Output compute(double rack_travel) const override {
        return {rack_travel / ratio_fl_, rack_travel / ratio_fr_};
    }

private:
    double ratio_fl_, ratio_fr_;
};

// Factory helpers
inline std::unique_ptr<ISteeringKinematics>
make_ratio_steering_kinematics(double steering_ratio) {
    return std::make_unique<RatioSteeringKinematics>(steering_ratio);
}

inline std::unique_ptr<ISteeringKinematics>
make_ackermann_kinematics(double ratio_fl, double ratio_fr) {
    return std::make_unique<AckermannKinematics>(ratio_fl, ratio_fr);
}

}  // namespace vdsim
