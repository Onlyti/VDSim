#pragma once

#include <cmath>
#include <vector>

namespace vdsim {

class DelayLine {
public:
    explicit DelayLine(double deadtime_s) : deadtime_s_(deadtime_s) {}

    double step(double value, double dt) {
        if (deadtime_s_ <= 0.0 || dt <= 0.0) return value;
        const int depth = static_cast<int>(std::lround(deadtime_s_ / dt));
        if (depth <= 0) return value;

        if (depth_ != depth) {
            buf_.assign(static_cast<std::size_t>(depth), 0.0);
            write_idx_ = 0;
            depth_ = depth;
        }

        const double out = buf_[static_cast<std::size_t>(write_idx_)];
        buf_[static_cast<std::size_t>(write_idx_)] = value;
        write_idx_ = (write_idx_ + 1) % depth_;
        return out;
    }

    void reset() {
        for (double& v : buf_) v = 0.0;
        write_idx_ = 0;
    }

private:
    double deadtime_s_;
    std::vector<double> buf_;
    int write_idx_ {0};
    int depth_ {0};
};

}  // namespace vdsim
