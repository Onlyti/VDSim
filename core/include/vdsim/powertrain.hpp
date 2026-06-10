#pragma once

// Engine + gearbox powertrain (Drivetrain v2). Opt-in via VehicleParams.powertrain;
// when disabled the legacy flat torque (max_motor_torque x final_drive) is used, so
// the default catalog preset and the ISO baseline are unchanged.
//
// This header holds the parameter / map types. The stateful coupling (engine RPM from
// wheel speed, gear shifting, reflected inertia) lives in the IDrivetrain module.

#include <algorithm>
#include <cmath>
#include <vector>

namespace vdsim {

// 2D engine torque map: peak torque [N m] as a function of (rpm, throttle).
// torque[i][j] is the torque at throttle_breaks[i], rpm_breaks[j]. The closed-throttle
// row (throttle 0) carries the engine-braking / motoring torque (negative). Bilinear
// interpolation; queries are clamped to the table domain (rpm to [first,last] break,
// throttle to [first,last] break).
struct EngineMap {
    std::vector<double> rpm_breaks;       // ascending [rpm]
    std::vector<double> throttle_breaks;  // ascending [-], typically 0..1
    std::vector<std::vector<double>> torque;  // [throttle_idx][rpm_idx] [N m]

    bool valid() const {
        if (rpm_breaks.size() < 2 || throttle_breaks.empty()) return false;
        if (torque.size() != throttle_breaks.size()) return false;
        for (const auto& row : torque)
            if (row.size() != rpm_breaks.size()) return false;
        return true;
    }

    // Bilinear interpolation; clamps rpm/throttle to the table domain.
    double eval(double rpm, double throttle) const {
        if (!valid()) return 0.0;
        const double r = clampv(rpm, rpm_breaks);
        const double t = clampv(throttle, throttle_breaks);
        std::size_t ri; double rf;
        std::size_t ti; double tf;
        locate(rpm_breaks, r, ri, rf);
        locate(throttle_breaks, t, ti, tf);
        // corners
        const double t00 = torque[ti][ri];
        const double t01 = torque[ti][ri + (rpm_breaks.size() > 1 ? 1 : 0)];
        const std::size_t ti1 = (throttle_breaks.size() > 1) ? ti + 1 : ti;
        const double t10 = torque[ti1][ri];
        const double t11 = torque[ti1][ri + (rpm_breaks.size() > 1 ? 1 : 0)];
        const double a = t00 + (t01 - t00) * rf;     // lower throttle row, interp in rpm
        const double b = t10 + (t11 - t10) * rf;     // upper throttle row
        return a + (b - a) * tf;
    }

  private:
    static double clampv(double x, const std::vector<double>& br) {
        return std::min(std::max(x, br.front()), br.back());
    }
    // Find interval index i (so br[i] <= x <= br[i+1]) and fraction f in [0,1].
    static void locate(const std::vector<double>& br, double x,
                       std::size_t& i, double& f) {
        if (br.size() < 2) { i = 0; f = 0.0; return; }
        i = 0;
        while (i + 2 < br.size() && x > br[i + 1]) ++i;
        const double lo = br[i], hi = br[i + 1];
        f = (hi > lo) ? (x - lo) / (hi - lo) : 0.0;
        f = std::min(std::max(f, 0.0), 1.0);
    }
};

struct EngineParams {
    double    idle_rpm    {800.0};
    double    redline_rpm {6500.0};
    double    inertia     {0.25};   // crank + flywheel [kg m^2]
    EngineMap map;
};

// Forward gear ratios (g = 1..N), reverse, final drive, mechanical efficiency, and the
// torque-interrupt duration during a shift (clutch open).
struct GearboxParams {
    std::vector<double> gear_ratios {{3.5, 2.1, 1.4, 1.0, 0.8}};
    double reverse_ratio {3.2};
    double final_drive   {4.1};
    double efficiency    {0.92};
    double shift_time    {0.3};     // [s]
};

// Whole engine+gearbox powertrain. Opt-in: `enabled` (set when a YAML `powertrain:`
// block is present). When false the legacy flat torque is used (baseline unchanged).
struct PowertrainParams {
    bool          enabled {false};
    EngineParams  engine;
    GearboxParams gearbox;

    // Declarative shift policy. Manual = follow the driver's commanded gear;
    // AutoRpmThreshold = built-in up/down-shift on engine RPM with hysteresis.
    // A programmatic ShiftPolicy (e.g. a Python callable) can override this on the
    // drivetrain module (D3).
    enum class ShiftMode { Manual, AutoRpmThreshold };
    ShiftMode shift_mode   {ShiftMode::Manual};
    double    upshift_rpm  {5500.0};
    double    downshift_rpm{1800.0};
    int       start_gear   {1};
};

}  // namespace vdsim
