// Text TX templates for the realtime comms router: JSON state and NMEA 0183 GGA.
//
// The canonical binary VDS1 framing lives in cosim_protocol.hpp; this header
// holds the human-readable templates a scene may select per channel with
// `template: json` / `template: nmea_gga` (see configs/comms/*.yaml).
// Header-only so the encoders are unit-testable without linking the server.
#pragma once

#include "cosim_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace vdsim::cosim {

// TX scratch buffer size used by the realtime server: must hold the binary VDS1
// state frame and the longest text template (JSON state ~340 B; GGA ~80 B).
constexpr size_t kTxBufBytes = 512;
static_assert(kTxBufBytes >= static_cast<size_t>(kStateBytes),
              "TX buffer must also fit a binary VDS1 state frame");

// Geodetic datum a comms channel's local ENU metres are referenced to.
// Scene YAML: `origin: {lat: 37.5, lon: 127.0, alt: 30.0}`. Default (0,0,0)
// keeps every existing scene working (Null Island) without a schema change.
struct GeodeticOrigin {
    double lat_deg {0.0};
    double lon_deg {0.0};
    double alt_m   {0.0};
};

// Result of enu_to_geodetic(): WGS84-referenced latitude/longitude + altitude.
struct Geodetic {
    double lat_deg {0.0};
    double lon_deg {0.0};
    double alt_m   {0.0};
};

// Synthetic GGA quality fields. VDSim models GNSS as an additive-noise position
// sensor (core/src/sensor_model.cpp) — there is no constellation, DOP or
// differential model — so these are fixed, clearly-labelled placeholders that
// keep the sentence parseable by real receivers/consumers.
constexpr int    kGgaFixQuality = 1;     // 1 = autonomous GPS fix
constexpr int    kGgaSatellites = 12;    // placeholder: no constellation model
constexpr double kGgaHdop       = 0.9;   // placeholder: no DOP model
constexpr double kGgaGeoidSep   = 0.0;   // placeholder: no geoid undulation model

/// @brief Canonicalise a scene TX template name to the encoder it selects.
/// @param templ Raw `template:` string from the scene comms spec.
/// @return "vds1", "json" or "nmea_gga"; empty string if the name is unknown.
/// @note `vds1_state` is the long-form alias of `vds1`; both map to the binary
///       VDS1 STATE frame. An empty input keeps the historical `vds1` default.
inline std::string canonical_tx_template(const std::string& templ) {
    if (templ.empty() || templ == "vds1" || templ == "vds1_state") return "vds1";
    if (templ == "json")     return "json";
    if (templ == "nmea_gga") return "nmea_gga";
    return std::string();
}

/// @brief NMEA 0183 checksum: XOR of every char strictly between '$' and '*'.
/// @param body Sentence body (talker+type+fields), without '$' or '*'.
/// @return 8-bit checksum, printed by nmea_sentence() as two uppercase hex digits.
inline uint8_t nmea_checksum(const std::string& body) {
    uint8_t sum = 0;
    for (const char c : body) sum ^= static_cast<uint8_t>(c);
    return sum;
}

/// @brief Frame comma-separated NMEA fields into a complete sentence.
/// @param fields fields[0] is the sentence id ("GPGGA"); the rest are data
///        fields in spec order. Empty strings emit empty (null) fields, which
///        is how NMEA encodes "not available".
/// @return "$<f0>,<f1>,...*<HH>\r\n" with an uppercase two-digit checksum.
inline std::string nmea_sentence(const std::vector<std::string>& fields) {
    std::string body;
    for (size_t i = 0; i < fields.size(); ++i) {
        if (i) body += ',';
        body += fields[i];
    }
    char tail[8];
    std::snprintf(tail, sizeof(tail), "*%02X\r\n", nmea_checksum(body));
    return "$" + body + tail;
}

/// @brief Format a signed angle as an unsigned NMEA degrees+minutes field.
/// @param deg Angle [deg]; the sign is dropped (the hemisphere field carries it).
/// @param deg_digits Integer degree digits: 2 for latitude, 3 for longitude.
/// @return "ddmm.mmmm" (or "dddmm.mmmm"), minutes zero-padded to 2 integer digits.
/// @note Minutes are rounded to the printed 4-decimal precision *before* the
///       range check so a value rounding up to 60.0000 carries into degrees
///       instead of emitting an illegal minute field.
inline std::string nmea_deg_min(double deg, int deg_digits) {
    const double a = std::fabs(deg);
    int    d = static_cast<int>(a);
    double m = std::round((a - d) * 60.0 * 1e4) / 1e4;
    if (m >= 60.0) { m -= 60.0; d += 1; }
    char out[32];
    if (deg_digits >= 3) std::snprintf(out, sizeof(out), "%03d%07.4f", d, m);
    else                 std::snprintf(out, sizeof(out), "%02d%07.4f", d, m);
    return std::string(out);
}

/// @brief Local ENU offset -> geodetic, equirectangular about the channel origin.
///
/// Projection (tangent-plane / "flat Earth" about lat0, WGS84 ellipsoid
/// a = 6378137 m, f = 1/298.257223563, e^2 = f(2-f)):
/// @verbatim
///   M  = a (1 - e^2) / (1 - e^2 sin^2(lat0))^(3/2)   meridian radius of curvature
///   Nr = a / sqrt(1 - e^2 sin^2(lat0))               prime-vertical radius
///   lat = lat0 + (north / M)  * 180/pi
///   lon = lon0 + (east / (Nr * cos(lat0))) * 180/pi
///   alt = alt0 + up
/// @endverbatim
///
/// Accuracy limits — this is NOT a WGS84-exact (Vincenty/geodesic) inverse:
///  - The radii of curvature are frozen at lat0, so the scale error grows with
///    distance from the origin. It is ~0 at the origin, of order 1e-5 relative
///    (~0.1 m in 10 km) at 10 km, and ~1e-3 relative (~100 m) at 100 km.
///  - Meridian convergence is ignored: the tangent plane's north is the origin's
///    north, not the local grid north.
///  - Degenerate near the poles: cos(lat0) -> 0 makes the longitude scale blow
///    up. Guarded below with a 1e-12 floor; results above ~|lat0| = 89.9 deg
///    are not meaningful.
/// This is adequate for a driving simulator whose scenes span metres to a few
/// kilometres; it must not be used as a survey-grade transformation.
///
/// @param east  ENU east offset from the origin [m].
/// @param north ENU north offset from the origin [m].
/// @param up    ENU up offset from the origin [m].
/// @param o     Geodetic datum the offsets are referenced to.
/// @return Latitude/longitude [deg] and altitude [m].
inline Geodetic enu_to_geodetic(double east, double north, double up,
                                const GeodeticOrigin& o) {
    constexpr double kA   = 6378137.0;                    // WGS84 semi-major axis [m]
    constexpr double kF   = 1.0 / 298.257223563;          // WGS84 flattening
    constexpr double kE2  = kF * (2.0 - kF);              // first eccentricity squared
    constexpr double kDeg = 57.29577951308232;            // 180/pi
    const double lat0 = o.lat_deg / kDeg;
    const double s    = std::sin(lat0);
    const double w2   = 1.0 - kE2 * s * s;
    const double w    = std::sqrt(w2);
    const double m_r  = kA * (1.0 - kE2) / (w2 * w);      // meridian radius
    const double n_r  = kA / w;                           // prime-vertical radius
    const double clat = std::max(std::fabs(std::cos(lat0)), 1e-12);
    Geodetic g;
    g.lat_deg = o.lat_deg + (north / m_r) * kDeg;
    g.lon_deg = o.lon_deg + (east / (n_r * clat)) * kDeg;
    g.alt_m   = o.alt_m + up;
    return g;
}

/// @brief Current UTC time as seconds since midnight, for the GGA time field.
/// @return Seconds of day in [0, 86400).
/// @note StateFields::timestamp cannot be used: the realtime server stamps it
///       from steady_clock (cosim/realtime_server.cpp now_s()), which is a
///       monotonic uptime, not a wall clock. A GGA sentence carries real UTC,
///       so the sentence is stamped from the system clock at encode time.
inline double utc_seconds_of_day_now() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const double secs = std::chrono::duration<double>(now).count();
    return secs - std::floor(secs / 86400.0) * 86400.0;
}

/// @brief Build a `$GPGGA` sentence for one vehicle state.
///
/// Position source: the *measured* GNSS channel (StateFields::m_gnss_x/_y), not
/// the truth pose (x/y). A GGA sentence is a receiver output, so a consumer must
/// see the same noise/bias the rest of the sensor stack sees. This is safe with
/// no sensor noise configured: SensorModel::apply() takes an identity path when
/// disabled and still fills gnss_x/gnss_y from the truth position
/// (core/src/sensor_model.cpp), so the field is never left unset — no fallback
/// to x/y is needed. Altitude uses the truth z, because SensorMeas models no
/// GNSS altitude channel.
///
/// Axis convention: VDSim world frame is ENU right-handed, X east / Y north /
/// Z up (documented at core/include/vdsim/coordinate.hpp:8), so x maps to the
/// ENU east offset and y to the north offset — no swap.
///
/// @param s State to encode; m_gnss_x/m_gnss_y are ENU metres from `o`.
/// @param o Per-channel geodetic datum for the ENU->lat/lon projection.
/// @param utc_sod UTC seconds since midnight for the time field (see
///        utc_seconds_of_day_now()); values outside [0,86400) are wrapped.
/// @return Complete sentence including the "*HH\r\n" terminator.
inline std::string encode_gga(const StateFields& s, const GeodeticOrigin& o,
                              double utc_sod) {
    const Geodetic g = enu_to_geodetic(s.m_gnss_x, s.m_gnss_y, s.z, o);
    double sod = utc_sod - std::floor(utc_sod / 86400.0) * 86400.0;
    const int hh = static_cast<int>(sod / 3600.0);
    const int mm = static_cast<int>((sod - hh * 3600.0) / 60.0);
    const double ss = sod - hh * 3600.0 - mm * 60.0;
    char buf[32];
    std::vector<std::string> f;
    f.reserve(15);
    f.emplace_back("GPGGA");
    std::snprintf(buf, sizeof(buf), "%02d%02d%05.2f", hh, mm, ss);
    f.emplace_back(buf);                                   // UTC hhmmss.ss
    f.emplace_back(nmea_deg_min(g.lat_deg, 2));            // ddmm.mmmm
    f.emplace_back(g.lat_deg >= 0.0 ? "N" : "S");
    f.emplace_back(nmea_deg_min(g.lon_deg, 3));            // dddmm.mmmm
    f.emplace_back(g.lon_deg >= 0.0 ? "E" : "W");
    std::snprintf(buf, sizeof(buf), "%d", kGgaFixQuality);
    f.emplace_back(buf);                                   // fix quality
    std::snprintf(buf, sizeof(buf), "%02d", kGgaSatellites);
    f.emplace_back(buf);                                   // satellites in use
    std::snprintf(buf, sizeof(buf), "%.1f", kGgaHdop);
    f.emplace_back(buf);                                   // HDOP
    std::snprintf(buf, sizeof(buf), "%.1f", g.alt_m);
    f.emplace_back(buf);
    f.emplace_back("M");                                   // altitude + unit
    std::snprintf(buf, sizeof(buf), "%.1f", kGgaGeoidSep);
    f.emplace_back(buf);
    f.emplace_back("M");                                   // geoid separation + unit
    f.emplace_back("");                                    // DGPS age (none)
    f.emplace_back("");                                    // DGPS station id (none)
    return nmea_sentence(f);
}

/// @brief Encode a state as compact one-line JSON for HTTP/MQTT forwarding.
///
/// {"id":0,"t":1.23,"x":45.6,"y":-3.2,"yaw":0.04,"vx":15.1,"vy":0.2,"r":0.01,
///  "ax":0.3,"ay":1.2,"Fz":[...],"alpha":[...],"kappa":[...]}
///
/// @param buf Output buffer (not NUL-terminated in the returned length).
/// @param buflen Capacity of @p buf in bytes.
/// @param s State to encode.
/// @return Byte count written on success; -1 if @p buf is too small. A partial
///         snprintf result is never reported as success: truncated JSON is
///         invalid JSON, and silently putting it on the wire would corrupt the
///         consumer's stream.
inline int encode_state_json(uint8_t* buf, size_t buflen, const StateFields& s) {
    char* p = reinterpret_cast<char*>(buf);
    const int n = std::snprintf(p, buflen,
        "{\"id\":%u,\"t\":%.3f,"
        "\"x\":%.2f,\"y\":%.2f,\"yaw\":%.4f,"
        "\"vx\":%.2f,\"vy\":%.2f,\"r\":%.4f,"
        "\"ax\":%.2f,\"ay\":%.2f,"
        "\"Fz\":[%.0f,%.0f,%.0f,%.0f],"
        "\"alpha\":[%.4f,%.4f,%.4f,%.4f],"
        "\"kappa\":[%.4f,%.4f,%.4f,%.4f]}",
        s.vehicle_id, s.timestamp,
        s.x, s.y, s.yaw,
        s.vx, s.vy, s.yaw_rate,
        s.ax, s.ay,
        s.Fz[0], s.Fz[1], s.Fz[2], s.Fz[3],
        s.slip_angle[0], s.slip_angle[1], s.slip_angle[2], s.slip_angle[3],
        s.slip_ratio[0], s.slip_ratio[1], s.slip_ratio[2], s.slip_ratio[3]);
    // snprintf returns the length it *would* have written: n >= buflen means the
    // text was cut short (and n < 0 is an encoding error).
    if (n < 0 || static_cast<size_t>(n) >= buflen) return -1;
    return n;
}

/// @brief Encode a state as a `$GPGGA` sentence into a raw byte buffer.
/// @param buf Output buffer.
/// @param buflen Capacity of @p buf in bytes.
/// @param s State to encode (see encode_gga() for the position-source rationale).
/// @param o Per-channel geodetic datum.
/// @param utc_sod UTC seconds since midnight for the time field.
/// @return Byte count written (excluding any NUL); -1 if @p buf is too small.
inline int encode_state_nmea_gga(uint8_t* buf, size_t buflen, const StateFields& s,
                                 const GeodeticOrigin& o, double utc_sod) {
    const std::string sentence = encode_gga(s, o, utc_sod);
    if (sentence.size() > buflen) return -1;
    std::memcpy(buf, sentence.data(), sentence.size());
    return static_cast<int>(sentence.size());
}

}  // namespace vdsim::cosim
