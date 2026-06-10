#pragma once

// Engine + gearbox powertrain (Drivetrain v2). Opt-in via VehicleParams.powertrain;
// when disabled the legacy flat torque (max_motor_torque x final_drive) is used, so
// the default catalog preset and the ISO baseline are unchanged.
//
// This header holds the parameter / map types. The stateful coupling (engine RPM from
// wheel speed, gear shifting, reflected inertia) lives in the IDrivetrain module.

#include <algorithm>
#include <cmath>
#include <functional>
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

// Inputs a shift policy sees each step. A policy returns the desired gear
// (1..num_gears forward, <0 reverse, 0 neutral); the gearbox enforces the shift
// time-out and only acts on a change.
struct ShiftContext {
    double engine_rpm;
    int    current_gear;
    double vehicle_speed;   // [m/s]
    double throttle;        // 0..1 (pedal opening)
    double brake;           // 0..1
    int    num_gears;
};
using ShiftPolicy = std::function<int(const ShiftContext&)>;

// Stateful engine + gearbox: derives engine RPM from the driven-wheel speed (idle
// floor + a slipping launch clutch at low speed), runs the shift policy with a
// shift-time torque interrupt, and returns the axle drive torque. Unit responsibility
// only — the IDrivetrain module owns wheel split and the dynamics own the EOM.
class EngineGearbox {
  public:
    void configure(const PowertrainParams& p) {
        p_ = p;
        reset();
    }
    void set_shift_policy(ShiftPolicy fn) { policy_ = std::move(fn); }

    void reset() {
        const int N = num_gears();
        gear_ = std::min(std::max(p_.start_gear, N ? 1 : 0), N ? N : 0);
        shift_timer_ = 0.0;
        rpm_ = p_.engine.idle_rpm;
    }

    // Advance one step. driven_omega: mean spin of the driven wheels [rad/s];
    // v: vehicle longitudinal speed [m/s]; commanded_gear: driver request (manual).
    // Returns the drive torque at the (combined) driven axle [N m].
    double axle_torque(double throttle, double brake, double driven_omega,
                       double v, int commanded_gear, double dt) {
        const auto& gb = p_.gearbox;
        const int N = num_gears();
        throttle = std::min(std::max(throttle, 0.0), 1.0);

        // --- engine RPM from wheel speed; idle floor + low-speed clutch slip ---
        const double ratio_now = gear_ratio(gear_);
        const double rpm_geom = std::abs(driven_omega) * std::abs(ratio_now)
                              * gb.final_drive * 60.0 / (2.0 * M_PI);
        const bool slipping = rpm_geom < p_.engine.idle_rpm;
        const double stall_rpm = std::clamp(0.5 * p_.engine.redline_rpm,
                                            p_.engine.idle_rpm, p_.engine.redline_rpm);
        const double rpm_target = slipping
            ? p_.engine.idle_rpm + throttle * (stall_rpm - p_.engine.idle_rpm)
            : rpm_geom;
        rpm_ = std::clamp(rpm_target, p_.engine.idle_rpm, p_.engine.redline_rpm);

        // --- shift logic (locked out for shift_time after a change) ---
        if (shift_timer_ > 0.0) {
            shift_timer_ = std::max(0.0, shift_timer_ - dt);
        } else {
            const int target = decide_gear(commanded_gear, v, brake, throttle);
            if (target != gear_ && std::abs(target) <= N && target != 0) {
                gear_ = target;
                shift_timer_ = gb.shift_time;
            } else if (target == 0) {       // neutral allowed
                gear_ = 0;
            }
        }

        // --- torque ---
        if (shift_timer_ > 0.0 || gear_ == 0) return 0.0;   // clutch open / neutral
        double T_eng = p_.engine.map.eval(rpm_, throttle);
        if (slipping) T_eng = std::max(0.0, T_eng);  // a slipping launch clutch does
                                                     // not transmit engine braking
        return T_eng * gear_ratio(gear_) * gb.final_drive * gb.efficiency;
    }

    double engine_rpm()   const { return rpm_; }
    int    current_gear() const { return gear_; }
    // Engine inertia reflected to the wheel through the current gear: I*(ratio*fd)^2.
    double reflected_inertia() const {
        if (gear_ == 0) return 0.0;
        const double r = gear_ratio(gear_) * p_.gearbox.final_drive;
        return p_.engine.inertia * r * r;
    }

  private:
    int num_gears() const { return static_cast<int>(p_.gearbox.gear_ratios.size()); }
    double gear_ratio(int g) const {
        if (g < 0)  return p_.gearbox.reverse_ratio;
        if (g == 0) return 0.0;
        const int idx = std::min(g, num_gears());
        return num_gears() ? p_.gearbox.gear_ratios[idx - 1] : 0.0;
    }
    int decide_gear(int commanded, double v, double brake, double throttle) {
        if (policy_) {
            return policy_(ShiftContext{rpm_, gear_, v, throttle, brake, num_gears()});
        }
        switch (p_.shift_mode) {
            case PowertrainParams::ShiftMode::Manual:
                return commanded;
            case PowertrainParams::ShiftMode::AutoRpmThreshold:
                if (gear_ <= 0) return commanded;     // out of auto range (R/N)
                if (rpm_ > p_.upshift_rpm  && gear_ < num_gears()) return gear_ + 1;
                if (rpm_ < p_.downshift_rpm && gear_ > 1)          return gear_ - 1;
                return gear_;
        }
        return gear_;
    }

    PowertrainParams p_;
    ShiftPolicy policy_;
    int    gear_ {1};
    double shift_timer_ {0.0};
    double rpm_ {800.0};
};

}  // namespace vdsim
