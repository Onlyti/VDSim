// CSV-backed lookup table for ISuspensionKinematics.
//
// CSV format produced by tools/kinematics/dw_3d_solver.py:
//    wheel_travel, steer_rack_dy, camber, toe, track_change, caster, valid
// Rows are expected on a regular grid (travels outer, steers inner).  Linear
// interpolation in both axes; nearest-neighbor clamp outside the table.

#include "vdsim/suspension.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace vdsim {

namespace {

struct Row {
    double travel, steer, camber, toe, track_change, caster;
    bool   valid;
};

class LookupKinematics final : public ISuspensionKinematics {
public:
    explicit LookupKinematics(std::vector<Row> rows) : rows_(std::move(rows)) {
        // Extract sorted unique travel/steer axes.
        std::set<double> ts, ss;
        for (const auto& r : rows_) { ts.insert(r.travel); ss.insert(r.steer); }
        travels_.assign(ts.begin(), ts.end());
        steers_.assign(ss.begin(), ss.end());
        // Index map: row indexed by (i_travel * n_steer + i_steer)
        n_t_ = travels_.size(); n_s_ = steers_.size();
        idx_.assign(n_t_ * n_s_, -1);
        for (size_t k = 0; k < rows_.size(); ++k) {
            const auto& r = rows_[k];
            int it = std::lower_bound(travels_.begin(), travels_.end(), r.travel) -
                     travels_.begin();
            int is = std::lower_bound(steers_.begin(), steers_.end(), r.steer) -
                     steers_.begin();
            if (it < (int)n_t_ && is < (int)n_s_ &&
                std::abs(travels_[it] - r.travel) < 1e-9 &&
                std::abs(steers_[is]  - r.steer)  < 1e-9) {
                idx_[it * n_s_ + is] = (int)k;
            }
        }
        spdlog::debug("[LookupKinematics] loaded {}x{} grid, {} rows",
                      n_t_, n_s_, rows_.size());
    }

    Output compute(double wheel_travel, double steer_input) const noexcept override {
        Output o;
        if (rows_.empty()) return o;

        // Bilinear interpolation, with clamp outside grid.
        auto bracket = [](const std::vector<double>& xs, double q,
                          int& i0, int& i1, double& t) {
            if (q <= xs.front()) { i0 = i1 = 0; t = 0.0; return; }
            if (q >= xs.back())  { i0 = i1 = (int)xs.size() - 1; t = 0.0; return; }
            i1 = std::upper_bound(xs.begin(), xs.end(), q) - xs.begin();
            i0 = i1 - 1;
            t = (q - xs[i0]) / (xs[i1] - xs[i0]);
        };
        int it0, it1, is0, is1;
        double ft, fs;
        bracket(travels_, wheel_travel, it0, it1, ft);
        bracket(steers_,  steer_input,  is0, is1, fs);

        auto get = [&](int it, int is) -> const Row* {
            const int k = idx_[it * n_s_ + is];
            return (k >= 0) ? &rows_[k] : nullptr;
        };
        const Row* r00 = get(it0, is0);
        const Row* r01 = get(it0, is1);
        const Row* r10 = get(it1, is0);
        const Row* r11 = get(it1, is1);
        if (!r00 || !r01 || !r10 || !r11) {
            // Sparse hole — fall back to nearest available row.
            for (auto* p : {r00, r01, r10, r11}) {
                if (p && p->valid) {
                    o.camber = p->camber; o.toe = p->toe;
                    o.track_change = p->track_change; o.caster = p->caster;
                    return o;
                }
            }
            return o;
        }
        auto bilerp = [&](double f00, double f01, double f10, double f11) {
            const double a = (1 - ft) * (1 - fs) * f00
                           + (1 - ft) * fs       * f01
                           + ft       * (1 - fs) * f10
                           + ft       * fs       * f11;
            return a;
        };
        o.camber       = bilerp(r00->camber,       r01->camber,
                                 r10->camber,       r11->camber);
        o.toe          = bilerp(r00->toe,          r01->toe,
                                 r10->toe,          r11->toe);
        o.track_change = bilerp(r00->track_change, r01->track_change,
                                 r10->track_change, r11->track_change);
        o.caster       = bilerp(r00->caster,       r01->caster,
                                 r10->caster,       r11->caster);
        return o;
    }

private:
    std::vector<Row> rows_;
    std::vector<double> travels_, steers_;
    std::vector<int> idx_;
    size_t n_t_ {0}, n_s_ {0};
};

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) out.push_back(tok);
    return out;
}

}  // namespace

std::unique_ptr<ISuspensionKinematics>
create_lookup_kinematics(const std::string& csv_path) {
    std::ifstream f(csv_path);
    if (!f) throw std::runtime_error("Cannot open CSV: " + csv_path);
    std::string header;
    if (!std::getline(f, header))
        throw std::runtime_error("Empty CSV: " + csv_path);

    std::vector<Row> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        const auto toks = split_csv(line);
        if (toks.size() < 7) continue;
        try {
            const int valid = std::stoi(toks[6]);
            if (!valid) continue;
            Row r;
            r.travel = std::stod(toks[0]);
            r.steer  = std::stod(toks[1]);
            r.camber = std::stod(toks[2]);
            r.toe    = std::stod(toks[3]);
            r.track_change = std::stod(toks[4]);
            r.caster = std::stod(toks[5]);
            r.valid  = true;
            rows.push_back(r);
        } catch (const std::exception&) {
            continue;
        }
    }
    if (rows.empty())
        throw std::runtime_error("CSV has no valid rows: " + csv_path);
    return std::make_unique<LookupKinematics>(std::move(rows));
}

}  // namespace vdsim
