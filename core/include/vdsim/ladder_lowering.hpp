#pragma once

#include <algorithm>
#include <cmath>

namespace vdsim {

// Legacy PoC calibration for control-ladder L1–L3 → L4 (throttle/brake) lowering.
// Values are frozen for ISO baseline compatibility; centralised here (was duplicated
// in dynamics + CascadeController).
struct LadderLoweringCalib {
    static constexpr double kRefMassKg        = 1500.0;
    static constexpr double kRefAccelMps2     = 5.0;
    static constexpr double kRefDriveTorqueNm = 600.0;
    static constexpr double kRefBrakeTorqueNm = 4000.0;
};

inline double fx_total_pedal_scale(double fx_total) {
    return fx_total / (LadderLoweringCalib::kRefMassKg * LadderLoweringCalib::kRefAccelMps2);
}

inline void motor_brake_torque_to_pedal(double t_drive, double t_brake,
                                        double& throttle, double& brake) {
    throttle = std::clamp(t_drive / LadderLoweringCalib::kRefDriveTorqueNm, 0.0, 1.0);
    brake    = std::clamp(t_brake / LadderLoweringCalib::kRefBrakeTorqueNm
                       - std::min(0.0, t_drive) / LadderLoweringCalib::kRefBrakeTorqueNm,
                          0.0, 1.0);
}

inline void motor_brake_torque_to_pedal_bicycle(double t_drive, double t_brake,
                                                double& throttle, double& brake) {
    throttle = std::clamp(t_drive / LadderLoweringCalib::kRefDriveTorqueNm, -1.0, 1.0);
    brake    = std::clamp(t_brake / LadderLoweringCalib::kRefBrakeTorqueNm, 0.0, 1.0);
    if (throttle < 0.0) {
        brake    = std::max(brake, -throttle);
        throttle = 0.0;
    }
}

inline void axle_torque_to_pedal(double t_drive, double t_brake,
                                 double& throttle, double& brake) {
    motor_brake_torque_to_pedal(t_drive, t_brake, throttle, brake);
}

inline void axle_torque_to_pedal_bicycle(double t_drive, double t_brake,
                                         double& throttle, double& brake) {
    motor_brake_torque_to_pedal_bicycle(t_drive, t_brake, throttle, brake);
}

inline void net_torque_to_pedal(double t_net, double& throttle, double& brake) {
    throttle = std::clamp(t_net / LadderLoweringCalib::kRefDriveTorqueNm, 0.0, 1.0);
    brake    = std::clamp(-t_net / LadderLoweringCalib::kRefBrakeTorqueNm, 0.0, 1.0);
}

}  // namespace vdsim
