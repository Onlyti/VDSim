// VDSim <-> AutoHYU UDP co-simulation wire protocol.
// See docs/vdsim_bridge_interface_requirements.md (§3-5).
//
// Fixed-length, little-endian, tightly-packed datagrams. 24-byte common header,
// then payload, then a trailing uint32 CRC32 over all preceding bytes.
// Serialization is byte-cursor based (host assumed little-endian = x86_64).
#pragma once

#include <cstdint>
#include <cstring>

namespace vdsim::cosim {

constexpr uint32_t kMagic       = 0x56445331u;  // "VDS1"
constexpr uint16_t kVersion     = 1;
constexpr uint16_t kMsgCmd      = 1;
constexpr uint16_t kMsgState    = 2;
constexpr int      kHeaderBytes = 24;
constexpr int      kCmdBytes    = 76;
constexpr int      kStateBytes  = 220;

// ---- CRC32 (IEEE 802.3, matches Python zlib.crc32) ----
inline uint32_t crc32(const uint8_t* d, size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) {
        crc ^= d[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
    }
    return ~crc;
}

// ---- little-endian byte cursor ----
struct Writer {
    uint8_t* p; size_t off {0};
    template <typename T> void put(T v) { std::memcpy(p + off, &v, sizeof(T)); off += sizeof(T); }
    void pad(int n) { std::memset(p + off, 0, n); off += n; }
};
struct Reader {
    const uint8_t* p; size_t off {0};
    template <typename T> T get() { T v; std::memcpy(&v, p + off, sizeof(T)); off += sizeof(T); return v; }
    void skip(int n) { off += n; }
};

// ---- field structs ----
struct CmdFields {
    double steer_tire {0.0};   // [rad]
    double throttle   {0.0};
    double brake      {0.0};
    int32_t gear      {1};
    uint8_t handbrake {0};
    double aux_accel  {0.0};
    double aux_speed  {0.0};
    uint32_t seq      {0};
    double timestamp  {0.0};
};

struct StateFields {
    double x{0}, y{0}, z{0};
    double roll{0}, pitch{0}, yaw{0};
    double vx{0}, vy{0}, vz{0};
    double roll_rate{0}, pitch_rate{0}, yaw_rate{0};
    double ax{0}, ay{0};
    double wheel_spin[4] {0,0,0,0};
    double steer_applied {0};
    double wheel_radius  {0};
    double Fz[4] {0,0,0,0};
    uint32_t seq {0};
    double timestamp {0};
};

inline void write_header(Writer& w, uint16_t msg_type, uint32_t seq, double ts) {
    w.put(kMagic); w.put(kVersion); w.put(msg_type); w.put(seq);
    w.put<uint32_t>(0);            // _pad
    w.put(ts);
}

// Encode a STATE packet into buf (must be >= kStateBytes). Returns byte count.
inline int encode_state(uint8_t* buf, const StateFields& s) {
    Writer w{buf};
    write_header(w, kMsgState, s.seq, s.timestamp);
    w.put(s.x); w.put(s.y); w.put(s.z);
    w.put(s.roll); w.put(s.pitch); w.put(s.yaw);
    w.put(s.vx); w.put(s.vy); w.put(s.vz);
    w.put(s.roll_rate); w.put(s.pitch_rate); w.put(s.yaw_rate);
    w.put(s.ax); w.put(s.ay);
    for (double v : s.wheel_spin) w.put(v);
    w.put(s.steer_applied); w.put(s.wheel_radius);
    for (double v : s.Fz) w.put(v);
    const uint32_t c = crc32(buf, w.off);   // crc over header+payload
    w.put(c);
    return static_cast<int>(w.off);
}

// Decode + validate a CMD packet. Returns false on size/magic/version/CRC error.
inline bool decode_cmd(const uint8_t* buf, size_t n, CmdFields& out) {
    if (n < static_cast<size_t>(kCmdBytes)) return false;
    Reader r{buf};
    if (r.get<uint32_t>() != kMagic) return false;
    if (r.get<uint16_t>() != kVersion) return false;
    if (r.get<uint16_t>() != kMsgCmd) return false;
    out.seq = r.get<uint32_t>();
    r.skip(4);                       // _pad
    out.timestamp = r.get<double>();
    out.steer_tire = r.get<double>();
    out.throttle   = r.get<double>();
    out.brake      = r.get<double>();
    out.gear       = r.get<int32_t>();
    out.handbrake  = r.get<uint8_t>();
    r.skip(3);                       // _pad
    out.aux_accel  = r.get<double>();
    out.aux_speed  = r.get<double>();
    const uint32_t want = r.get<uint32_t>();
    const uint32_t have = crc32(buf, kCmdBytes - 4);
    return want == have;
}

}  // namespace vdsim::cosim
