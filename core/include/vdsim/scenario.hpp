#pragma once

#include <string>
#include <vector>

#include "vdsim/control.hpp"

namespace vdsim {

// Time-stamped control sample.
struct ControlSample {
    double t        {0.0};   // [s]
    double throttle {0.0};   // [0, 1]
    double brake    {0.0};   // [0, 1]
    double steer    {0.0};   // wheel angle [rad]
    int    gear     {1};     // 1 forward, -1 reverse
};

struct MuSample {
    double t  {0.0};
    double mu {1.0};
};

struct Scenario {
    enum class Interp { ZOH, Linear };

    std::string  name            {"unnamed"};
    double       initial_vx      {10.0};   // [m/s]
    double       duration        {5.0};    // [s]
    double       dt              {0.005};  // [s] outer tick
    double       mu              {1.0};    // surface friction multiplier (fallback if mu_profile empty)
    Interp       interpolation   {Interp::ZOH};
    std::vector<ControlSample> controls;   // sorted ascending by t
    std::vector<MuSample>      mu_profile; // optional time-varying mu

    static Scenario from_yaml(const std::string& path);
    void           to_yaml(const std::string& path) const;

    // Returns the control command at time t, applying the chosen interpolation.
    ControlSample sample(double t) const;
    // Returns mu at time t (uses mu_profile if non-empty, else fallback mu).
    double        sample_mu(double t) const;
};

}  // namespace vdsim
