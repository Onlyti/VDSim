#include "vdsim/snapshot.hpp"

#include <string>

namespace vdsim {
namespace snap {

void put_header(std::vector<double>& v) {
    v.push_back(kFormatMagic);
    v.push_back(static_cast<double>(kFormatVersion));
}

void check_header(const std::vector<double>& v, std::size_t& p) {
    const std::string reads = "this build reads v" + std::to_string(kFormatVersion);
    if (v.size() < 2 || v[0] != kFormatMagic)
        throw std::runtime_error(
            "SessionSnapshot: snapshot format v1 (no format header), " + reads +
            "; re-take the snapshot with this build");
    const double ver = v[1];
    if (ver != static_cast<double>(kFormatVersion))
        throw std::runtime_error(
            "SessionSnapshot: snapshot format v" +
            std::to_string(static_cast<long long>(ver)) + ", " + reads +
            "; re-take the snapshot with this build");
    p = 2;
}

// State is written field by field (not memcpy'd) so a snapshot stays readable
// across compilers and padding changes.
void put_state(std::vector<double>& v, const State& s) {
    for (int k = 0; k < 3; ++k) v.push_back(s.position[k]);
    v.push_back(s.orientation.w());
    v.push_back(s.orientation.x());
    v.push_back(s.orientation.y());
    v.push_back(s.orientation.z());
    for (int k = 0; k < 3; ++k) v.push_back(s.velocity[k]);
    for (int k = 0; k < 3; ++k) v.push_back(s.angular_velocity[k]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(s.wheel_spin[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(s.susp_compression[i]);
    for (int i = 0; i < NUM_WHEELS; ++i) v.push_back(s.susp_velocity[i]);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) v.push_back(s.unsprung_pos[i][k]);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) v.push_back(s.unsprung_vel[i][k]);
    v.push_back(s.rack_travel);
    v.push_back(s.rack_velocity);
}

State get_state(const std::vector<double>& v, std::size_t& p) {
    State s;
    for (int k = 0; k < 3; ++k) s.position[k] = get(v, p);
    const double qw = get(v, p), qx = get(v, p), qy = get(v, p), qz = get(v, p);
    s.orientation = Quat(qw, qx, qy, qz);
    for (int k = 0; k < 3; ++k) s.velocity[k] = get(v, p);
    for (int k = 0; k < 3; ++k) s.angular_velocity[k] = get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) s.wheel_spin[i] = get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) s.susp_compression[i] = get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i) s.susp_velocity[i] = get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) s.unsprung_pos[i][k] = get(v, p);
    for (int i = 0; i < NUM_WHEELS; ++i)
        for (int k = 0; k < 3; ++k) s.unsprung_vel[i][k] = get(v, p);
    s.rack_travel   = get(v, p);
    s.rack_velocity = get(v, p);
    return s;
}

}  // namespace snap
}  // namespace vdsim
