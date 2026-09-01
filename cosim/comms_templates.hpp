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
// lat_deg is always in [-90, 90] and lon_deg in (-180, 180]: the closed-form
// inverse produces both from atan2(), so no separate wrapping step is needed.
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

// Largest ENU offset a GGA position is reported for. Two-and-a-half Earth
// circumferences from the datum is not a vehicle, it is a diverged solver (or
// an uninitialised state), so the sentence reports "no fix" (quality 0, empty
// position fields) instead of a plausible-looking coordinate.
constexpr double kMaxEnuOffsetM = 1.0e8;

// Largest |altitude| printed in the GGA altitude field. Outside this the field
// is emitted empty ("not available"): "%.1f" of, say, 1e300 would need 300+
// characters and could not be formatted into a fixed-size field buffer.
constexpr double kMaxGgaAltitudeM = 1.0e7;

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
/// @param deg Angle [deg]; the sign is dropped (the hemisphere field carries
///        it). Must already be range-reduced: |deg| <= 180.
/// @param deg_digits Integer degree digits: 2 for latitude, 3 for longitude.
/// @return "ddmm.mmmm" (or "dddmm.mmmm"), minutes zero-padded to 2 integer
///         digits; an empty string (the NMEA "not available" field) when @p deg
///         is not finite or is out of range.
/// @note Minutes are rounded to the printed 4-decimal precision *before* the
///       range check so a value rounding up to 60.0000 carries into degrees
///       instead of emitting an illegal minute field.
/// @note The finiteness/range guard is what keeps the `static_cast<int>` below
///       defined: casting NaN or 1e18 to int is undefined behaviour, and the
///       observed result was a field like "-2147483648    nan" — with embedded
///       spaces inside a comma-delimited field — carrying a valid checksum.
inline std::string nmea_deg_min(double deg, int deg_digits) {
    if (!std::isfinite(deg) || std::fabs(deg) > 180.0) return std::string();
    const double a = std::fabs(deg);
    int    d = static_cast<int>(a);
    double m = std::round((a - d) * 60.0 * 1e4) / 1e4;
    if (m >= 60.0) { m -= 60.0; d += 1; }
    char out[32];
    if (deg_digits >= 3) std::snprintf(out, sizeof(out), "%03d%07.4f", d, m);
    else                 std::snprintf(out, sizeof(out), "%02d%07.4f", d, m);
    return std::string(out);
}

/// @brief Format seconds-of-day as the NMEA UTC time field "hhmmss.ss".
/// @param utc_sod UTC seconds since midnight; values outside [0, 86400) wrap.
/// @return "hhmmss.ss", or an empty field when @p utc_sod is not finite.
/// @note The value is rounded to the printed 1/100 s *first*, then split, so a
///       time in [59.995, 60) carries seconds -> minutes -> hours -> day rather
///       than printing the illegal "125960.00". Formatting "%05.2f" of the
///       unrounded seconds got this wrong about one tick in 12000 at 200 Hz.
inline std::string nmea_utc_field(double utc_sod) {
    if (!std::isfinite(utc_sod)) return std::string();
    constexpr double kCentisPerDay = 8640000.0;          // 86400 s * 100
    const double sod = utc_sod - std::floor(utc_sod / 86400.0) * 86400.0;
    double cs = std::floor(sod * 100.0 + 0.5);           // hundredths of a second
    if (cs >= kCentisPerDay) cs -= kCentisPerDay;        // 23:59:59.999 -> 00:00:00
    const int total = static_cast<int>(cs);
    char out[16];
    std::snprintf(out, sizeof(out), "%02d%02d%05.2f",
                  total / 360000, (total / 6000) % 60, (total % 6000) / 100.0);
    return std::string(out);
}

/// @brief Local ENU offset -> geodetic: exact closed-form WGS84 inverse.
///
/// The offsets are interpreted as WGS84 local Cartesian (topocentric) ENU about
/// the datum and inverted in three exact steps — there is no small-angle or
/// flat-Earth approximation anywhere in the chain:
/// @verbatim
///   1. datum -> ECEF:  N0 = a / sqrt(1 - e^2 sin^2 lat0)
///                      [x0 y0 z0] = [(N0+h0) cos lat0 cos lon0,
///                                    (N0+h0) cos lat0 sin lon0,
///                                    (N0(1-e^2)+h0) sin lat0]
///   2. ENU -> ECEF:    [x y z] = [x0 y0 z0] + R(lat0,lon0)^T [east north up]
///   3. ECEF -> geodetic: Bowring's closed form + Newton refinement.
/// @endverbatim
///
/// Accuracy: exact to double-precision rounding. Measured against PROJ's
/// `+proj=topocentric` / `+proj=cart` pipeline (pyproj 3.5 / PROJ 9) the worst
/// disagreement over the datum/offset grid in
/// tests/unit/test_comms_templates.cpp is 6e-8 m — sub-micrometre, at any
/// offset and any latitude, poles and antimeridian included. The
/// equirectangular form this replaced froze the longitude scale at cos(lat0)
/// and was off by 6.75 m at 10 km / 678 m at 100 km from the Seoul datum
/// (lat 37.5665), not the "0.1 m / 100 m" it advertised — that figure only
/// held near the equator, and the error scaled with tan(lat0).
///
/// What still limits the result is the *definition* of ENU, not this code:
///  - ENU is a flat tangent plane, so a purely horizontal offset d sits
///    ~d^2/(2R) above the ellipsoid: 7.8 mm at 10 m, 7.8 m at 10 km. That
///    appears in alt_m and is the geometrically correct answer for a
///    tangent-plane frame.
///  - Tangent-plane distance is not geodesic arc length (and tangent-plane
///    north is the datum's north, not local grid north), so the ground track
///    differs from a true WGS84 geodesic by ~d^3/(3 R^2): measured 0.008 m at
///    10 km and 8.2 m at 100 km. That is a property of asking for ENU, not a
///    defect of this inverse.
///  - Altitudes are ellipsoidal (HAE), not orthometric: VDSim has no geoid
///    model, which is why kGgaGeoidSep is a 0.0 placeholder.
///
/// @param east  ENU east offset from the origin [m].
/// @param north ENU north offset from the origin [m].
/// @param up    ENU up offset from the origin [m].
/// @param o     Geodetic datum the offsets are referenced to.
/// @return Latitude in [-90, 90] deg, longitude in (-180, 180] deg, ellipsoidal
///         altitude [m]. Non-finite inputs propagate as non-finite outputs; the
///         GGA encoder checks for that before formatting anything.
inline Geodetic enu_to_geodetic(double east, double north, double up,
                                const GeodeticOrigin& o) {
    constexpr double kA   = 6378137.0;                    // WGS84 semi-major axis [m]
    constexpr double kF   = 1.0 / 298.257223563;          // WGS84 flattening
    constexpr double kE2  = kF * (2.0 - kF);              // first eccentricity squared
    constexpr double kB   = kA * (1.0 - kF);              // semi-minor axis [m]
    constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB);  // second eccentricity sq.
    constexpr double kRad = 0.017453292519943295;         // pi/180
    constexpr double kDeg = 57.29577951308232;            // 180/pi

    const double lat0 = o.lat_deg * kRad;
    const double lon0 = o.lon_deg * kRad;
    const double sp = std::sin(lat0), cp = std::cos(lat0);
    const double sl = std::sin(lon0), cl = std::cos(lon0);

    const double n0 = kA / std::sqrt(1.0 - kE2 * sp * sp);
    const double x0 = (n0 + o.alt_m) * cp * cl;
    const double y0 = (n0 + o.alt_m) * cp * sl;
    const double z0 = (n0 * (1.0 - kE2) + o.alt_m) * sp;

    // R(lat0,lon0)^T, the transpose of the ECEF->ENU rotation at the datum.
    const double x = x0 - sl * east - sp * cl * north + cp * cl * up;
    const double y = y0 + cl * east - sp * sl * north + cp * sl * up;
    const double z = z0 +             cp      * north + sp      * up;

    Geodetic g;
    const double p = std::hypot(x, y);
    if (!(p > 1.0e-9)) {          // on the spin axis (or NaN): longitude undefined
        g.lat_deg = std::isfinite(z) ? (z >= 0.0 ? 90.0 : -90.0) : z;
        g.lon_deg = o.lon_deg - std::floor((o.lon_deg + 180.0) / 360.0) * 360.0;
        g.alt_m   = std::isfinite(z) ? std::fabs(z) - kB : z;
        return g;
    }
    // Bowring's parametric-latitude start value: already good to ~0.1 mm.
    const double th = std::atan2(z * kA, p * kB);
    const double st = std::sin(th), ct = std::cos(th);
    double lat = std::atan2(z + kEp2 * kB * st * st * st,
                            p - kE2  * kA * ct * ct * ct);
    // Two Newton steps drive the residual down to double-precision noise, so a
    // zero offset round-trips back to the datum to within ~1e-13 deg.
    for (int i = 0; i < 2; ++i) {
        const double s1 = std::sin(lat), c1 = std::cos(lat);
        const double w  = std::sqrt(1.0 - kE2 * s1 * s1);
        const double nr = kA / w;
        const double h  = p * c1 + z * s1 - kA * w;
        const double den = nr + h;
        if (!(std::fabs(den) > 1.0)) break;      // degenerate: keep Bowring's value
        lat = std::atan2(z, p * (1.0 - kE2 * nr / den));
    }
    const double s1 = std::sin(lat), c1 = std::cos(lat);
    g.lat_deg = lat * kDeg;
    g.lon_deg = std::atan2(y, x) * kDeg;
    g.alt_m   = p * c1 + z * s1 - kA * std::sqrt(1.0 - kE2 * s1 * s1);
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
/// Degenerate states: a diverged solver can hand the encoder NaN, infinity or
/// an absurd offset. Formatting those was undefined behaviour and produced a
/// checksum-valid sentence with spaces embedded inside comma-delimited fields.
/// Instead the sentence now degrades the way a real receiver does — quality 0,
/// 00 satellites, 99.9 HDOP and empty position/altitude fields — so it stays
/// well-formed while telling the consumer, explicitly, that there is no fix.
///
/// @param s State to encode; m_gnss_x/m_gnss_y are ENU metres from `o`.
/// @param o Per-channel geodetic datum for the ENU->lat/lon projection.
/// @param utc_sod UTC seconds since midnight for the time field (see
///        utc_seconds_of_day_now()); values outside [0,86400) are wrapped.
/// @return Complete sentence including the "*HH\r\n" terminator.
inline std::string encode_gga(const StateFields& s, const GeodeticOrigin& o,
                              double utc_sod) {
    const bool enu_ok = std::isfinite(s.m_gnss_x) && std::isfinite(s.m_gnss_y) &&
                        std::isfinite(s.z) &&
                        std::fabs(s.m_gnss_x) <= kMaxEnuOffsetM &&
                        std::fabs(s.m_gnss_y) <= kMaxEnuOffsetM &&
                        std::fabs(s.z)        <= kMaxEnuOffsetM &&
                        std::isfinite(o.lat_deg) && std::isfinite(o.lon_deg) &&
                        std::isfinite(o.alt_m);
    const Geodetic g = enu_ok ? enu_to_geodetic(s.m_gnss_x, s.m_gnss_y, s.z, o)
                              : Geodetic{};
    // nmea_deg_min() returns an empty field for anything it cannot represent,
    // so a bad projection result cannot leak into the sentence either.
    const std::string lat_f = enu_ok ? nmea_deg_min(g.lat_deg, 2) : std::string();
    const std::string lon_f = enu_ok ? nmea_deg_min(g.lon_deg, 3) : std::string();
    const bool fix_ok = !lat_f.empty() && !lon_f.empty();
    const bool alt_ok = fix_ok && std::isfinite(g.alt_m) &&
                        std::fabs(g.alt_m) <= kMaxGgaAltitudeM;

    char buf[32];
    std::vector<std::string> f;
    f.reserve(15);
    f.emplace_back("GPGGA");
    f.emplace_back(nmea_utc_field(utc_sod));               // UTC hhmmss.ss
    f.emplace_back(fix_ok ? lat_f : std::string());        // ddmm.mmmm
    f.emplace_back(fix_ok ? (g.lat_deg >= 0.0 ? "N" : "S") : "");
    f.emplace_back(fix_ok ? lon_f : std::string());        // dddmm.mmmm
    f.emplace_back(fix_ok ? (g.lon_deg >= 0.0 ? "E" : "W") : "");
    std::snprintf(buf, sizeof(buf), "%d", fix_ok ? kGgaFixQuality : 0);
    f.emplace_back(buf);                                   // fix quality (0 = invalid)
    std::snprintf(buf, sizeof(buf), "%02d", fix_ok ? kGgaSatellites : 0);
    f.emplace_back(buf);                                   // satellites in use
    std::snprintf(buf, sizeof(buf), "%.1f", fix_ok ? kGgaHdop : 99.9);
    f.emplace_back(buf);                                   // HDOP
    if (alt_ok) std::snprintf(buf, sizeof(buf), "%.1f", g.alt_m);
    f.emplace_back(alt_ok ? std::string(buf) : std::string());
    f.emplace_back("M");                                   // altitude + unit
    std::snprintf(buf, sizeof(buf), "%.1f", kGgaGeoidSep);
    f.emplace_back(fix_ok ? std::string(buf) : std::string());
    f.emplace_back("M");                                   // geoid separation + unit
    f.emplace_back("");                                    // DGPS age (none)
    f.emplace_back("");                                    // DGPS station id (none)
    return nmea_sentence(f);
}

/// @brief Format one double as a JSON number token.
/// @param v Value to format.
/// @param prec Digits after the decimal point.
/// @return The fixed-point text, or the literal `null` when @p v is not finite.
/// @note RFC 8259 has no NaN/Infinity literals: printf's "nan"/"inf" produce a
///       document that neither json.loads nor JSON.parse accepts, and such a
///       frame is short enough that the truncation guard never catches it.
///       `null` is chosen over dropping the frame deliberately — telemetry
///       matters most at the moment the solver diverges, and a parseable object
///       with null fields says "this channel went bad", whereas a missing
///       datagram is indistinguishable from ordinary UDP loss.
inline std::string json_number(double v, int prec) {
    if (!std::isfinite(v)) return "null";
    char out[32];
    const int n = std::snprintf(out, sizeof(out), "%.*f", prec, v);
    if (n < 0) return "null";
    if (static_cast<size_t>(n) < sizeof(out)) return std::string(out, n);
    // Enormous-but-finite magnitudes need more digits than the scratch holds;
    // they will normally overflow the TX buffer and be dropped by the caller,
    // but they must never be reported as a silently truncated number.
    std::string big(static_cast<size_t>(n) + 1, '\0');
    std::snprintf(&big[0], big.size(), "%.*f", prec, v);
    big.resize(static_cast<size_t>(n));
    return big;
}

/// @brief Format n doubles as the body of a JSON array ("a,b,c,d").
/// @param v Pointer to the values.
/// @param n Number of values.
/// @param prec Digits after the decimal point, per json_number().
/// @return Comma-joined number tokens, without the enclosing brackets.
inline std::string json_number_array(const double* v, size_t n, int prec) {
    std::string out;
    for (size_t i = 0; i < n; ++i) {
        if (i) out += ',';
        out += json_number(v[i], prec);
    }
    return out;
}

/// @brief Encode a state as compact one-line JSON for HTTP/MQTT forwarding.
///
/// {"id":0,"t":1.23,"x":45.6,"y":-3.2,"yaw":0.04,"vx":15.1,"vy":0.2,"r":0.01,
///  "ax":0.3,"ay":1.2,"Fz":[...],"alpha":[...],"kappa":[...]}
///
/// Every numeric field goes through json_number(), so a non-finite value is
/// emitted as `null` and the object stays parseable (rationale there).
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
    const std::string fz    = json_number_array(s.Fz, 4, 0);
    const std::string alpha = json_number_array(s.slip_angle, 4, 4);
    const std::string kappa = json_number_array(s.slip_ratio, 4, 4);
    const int n = std::snprintf(p, buflen,
        "{\"id\":%u,\"t\":%s,"
        "\"x\":%s,\"y\":%s,\"yaw\":%s,"
        "\"vx\":%s,\"vy\":%s,\"r\":%s,"
        "\"ax\":%s,\"ay\":%s,"
        "\"Fz\":[%s],"
        "\"alpha\":[%s],"
        "\"kappa\":[%s]}",
        s.vehicle_id, json_number(s.timestamp, 3).c_str(),
        json_number(s.x, 2).c_str(), json_number(s.y, 2).c_str(),
        json_number(s.yaw, 4).c_str(),
        json_number(s.vx, 2).c_str(), json_number(s.vy, 2).c_str(),
        json_number(s.yaw_rate, 4).c_str(),
        json_number(s.ax, 2).c_str(), json_number(s.ay, 2).c_str(),
        fz.c_str(), alpha.c_str(), kappa.c_str());
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
