// Unit tests for the text TX templates (cosim/comms_templates.hpp):
// template canonicalisation, NMEA 0183 checksum/framing, ENU->geodetic
// projection, GGA field layout and the JSON state encoder.
//
// Reference values marked "PROJ reference" were produced on the build host with
// pyproj 3.5 / PROJ 9 by an implementation independent of the header under
// test: EPSG:4979 -> EPSG:4978 for the datum, the textbook ENU rotation, then
// EPSG:4978 -> EPSG:4979 for the inverse. They were cross-checked against
// PROJ's own `+proj=topocentric` operation, which agreed to every printed
// digit. The generator lives in the review evidence, not in the build; it needs
// pyproj, which is not a VDSim dependency.
#include <gtest/gtest.h>

#include "comms_templates.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using vdsim::cosim::GeodeticOrigin;
using vdsim::cosim::StateFields;

namespace {

// WGS84 radii of curvature at the equator. These were once treated as "exactly
// one degree" of offset; under the exact ENU->ECEF->geodetic inverse they are
// simply two convenient ~110 km offsets, and the tests below pin the exact
// latitude/longitude they actually map to.
constexpr double kMeridianDegAtEq = 110574.2727;   // meridian scale at lat0 = 0 [m/deg]
constexpr double kPrimeVertDegAtEq = 111319.4908;  // prime-vertical scale at lat0 = 0 [m/deg]

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

// Split "$BODY*HH\r\n" into its body and its checksum digits.
std::pair<std::string, std::string> split_sentence(const std::string& s) {
    const auto star = s.rfind('*');
    EXPECT_NE(star, std::string::npos);
    return {s.substr(1, star - 1), s.substr(star + 1, 2)};
}

// Comma-split an NMEA sentence body into fields.
std::vector<std::string> fields_of(const std::string& body) {
    std::vector<std::string> out;
    size_t i = 0;
    while (true) {
        const auto c = body.find(',', i);
        if (c == std::string::npos) { out.push_back(body.substr(i)); break; }
        out.push_back(body.substr(i, c - i));
        i = c + 1;
    }
    return out;
}

// Parse "$...*HH\r\n" into fields, asserting the checksum digits are correct.
std::vector<std::string> parse_checked_sentence(const std::string& sentence) {
    EXPECT_EQ(sentence.substr(sentence.size() - 2), "\r\n");
    const auto [body, hex] = split_sentence(sentence.substr(0, sentence.size() - 2));
    char expect[4];
    std::snprintf(expect, sizeof(expect), "%02X", vdsim::cosim::nmea_checksum(body));
    EXPECT_EQ(hex, std::string(expect)) << sentence;
    return fields_of(body);
}

}  // namespace

TEST(CommsTemplates, CanonicalTxTemplate) {
    EXPECT_EQ(vdsim::cosim::canonical_tx_template("vds1"), "vds1");
    EXPECT_EQ(vdsim::cosim::canonical_tx_template("vds1_state"), "vds1");
    EXPECT_EQ(vdsim::cosim::canonical_tx_template(""), "vds1");
    EXPECT_EQ(vdsim::cosim::canonical_tx_template("json"), "json");
    EXPECT_EQ(vdsim::cosim::canonical_tx_template("nmea_gga"), "nmea_gga");
    // Genuinely unknown templates must stay unknown so the server can skip them.
    EXPECT_TRUE(vdsim::cosim::canonical_tx_template("nmea_rmc").empty());
    EXPECT_TRUE(vdsim::cosim::canonical_tx_template("vds1_cmd").empty());
    EXPECT_TRUE(vdsim::cosim::canonical_tx_template("JSON").empty());
}

// Hand-checkable reference from the NMEA 0183 GGA literature: the XOR of every
// character between '$' and '*' of this sentence is 0x47.
TEST(CommsTemplates, ChecksumMatchesReferenceSentence) {
    const std::string body =
        "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    EXPECT_EQ(vdsim::cosim::nmea_checksum(body), 0x47);
    // XOR is self-inverse: folding the checksum back in must yield zero.
    uint8_t sum = vdsim::cosim::nmea_checksum(body);
    for (char c : body) sum ^= static_cast<uint8_t>(c);
    EXPECT_EQ(sum, 0);
}

TEST(CommsTemplates, SentenceFramingAndChecksumDigits) {
    const std::string s = vdsim::cosim::nmea_sentence({"GPGGA", "1", "", "2"});
    EXPECT_EQ(s.substr(0, 6), "$GPGGA");
    EXPECT_EQ(s.substr(s.size() - 2), "\r\n");
    const auto [body, hex] = split_sentence(s.substr(0, s.size() - 2));
    EXPECT_EQ(body, "GPGGA,1,,2");
    EXPECT_EQ(hex.size(), 2u);
    for (char c : hex) EXPECT_TRUE((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) << hex;
    char expect[4];
    std::snprintf(expect, sizeof(expect), "%02X", vdsim::cosim::nmea_checksum(body));
    EXPECT_EQ(hex, std::string(expect));
}

TEST(CommsTemplates, DegMinFormatting) {
    // 48.1173 deg -> 48 deg 07.038 min (the reference sentence's latitude).
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(48.1173, 2), "4807.0380");
    // Sign is dropped: the hemisphere field carries it.
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(-48.1173, 2), "4807.0380");
    // Longitude uses 3 degree digits, and minutes keep 2 padded integer digits.
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(11.5166666667, 3), "01131.0000");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(0.0, 2), "0000.0000");
    // A minute value that rounds up to 60.0000 must carry into the degrees.
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(12.99999999, 2), "1300.0000");
}

// Guards the UB that produced "$GPGGA,010000.00,-2147483648    nan,S,..." —
// static_cast<int> of NaN / 1e18 — and the embedded spaces that came with it.
TEST(CommsTemplates, DegMinRejectsNonFiniteAndOutOfRange) {
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(kNaN, 2), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(kInf, 3), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(-kInf, 2), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(1.0e18, 3), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(180.0001, 3), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(-180.0001, 3), "");
    // The extreme legal values are still formatted.
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(90.0, 2), "9000.0000");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(-180.0, 3), "18000.0000");
}

TEST(CommsTemplates, EnuToGeodeticZeroOffsetIsExactlyTheOrigin) {
    const GeodeticOrigin o{37.5665, 126.9780, 38.0};
    const auto g = vdsim::cosim::enu_to_geodetic(0.0, 0.0, 0.0, o);
    EXPECT_DOUBLE_EQ(g.lat_deg, o.lat_deg);
    EXPECT_DOUBLE_EQ(g.lon_deg, o.lon_deg);
    // Altitude goes through the ECEF round trip (h ~ 6.4e6 m of cancellation),
    // so it returns to the datum to sub-nanometre, not to the last bit.
    EXPECT_NEAR(g.alt_m, o.alt_m, 1.0e-8);
}

TEST(CommsTemplates, EnuToGeodeticKnownOffsetAtEquator) {
    const GeodeticOrigin o{0.0, 0.0, 0.0};
    const auto g = vdsim::cosim::enu_to_geodetic(kPrimeVertDegAtEq,
                                                 kMeridianDegAtEq, 12.5, o);
    // PROJ reference for east/north ~110 km with up = 12.5 m.
    EXPECT_NEAR(g.lat_deg, 0.999742207905550, 1e-9);
    EXPECT_NEAR(g.lon_deg, 0.999896520255518, 1e-9);
    // ENU is a flat tangent plane: 156 km out along it, the point sits ~1936 m
    // above the ellipsoid even though `up` was only 12.5 m.
    EXPECT_NEAR(g.alt_m, 1948.588722937, 1e-3);
    // Sign convention: +east raises longitude, +north raises latitude.
    const auto neg = vdsim::cosim::enu_to_geodetic(-kPrimeVertDegAtEq,
                                                   -kMeridianDegAtEq, 0.0, o);
    EXPECT_NEAR(neg.lat_deg, -0.999744179416095, 1e-9);
    EXPECT_NEAR(neg.lon_deg, -0.999898479474795, 1e-9);
    EXPECT_NEAR(neg.alt_m, 1936.092528899, 1e-3);
}

// D1 accuracy pin: away from the equator the old equirectangular form was off
// by 6.75 m at 10 km (lat 37.5665) and 15.2 m at 10 km (lat 60), because it
// froze the longitude scale at cos(lat0). The exact inverse must agree with the
// PROJ reference to well under a millimetre at both latitudes.
TEST(CommsTemplates, EnuToGeodeticMatchesProjReferenceAtTenKilometres) {
    struct Case {
        GeodeticOrigin o;
        double east, north;
        double lat_ref, lon_ref, alt_ref;
    };
    const Case cases[] = {
        {{37.5665, 126.9780, 38.0}, 10000.0,     0.0,
         37.566445740513110, 127.091189257618538, 45.829468708},
        {{37.5665, 126.9780, 38.0},     0.0, 10000.0,
         37.656598293327995, 126.977999999999994, 45.862581023},
        {{60.0,     10.0,     0.0}, 10000.0,     0.0,
         59.999878434844135,  10.179210880059555,  7.819570524},
        {{60.0,     10.0,     0.0},     0.0, 10000.0,
         60.089756019112222,  10.000000000000002,  7.832709746},
    };
    for (const Case& c : cases) {
        const auto g = vdsim::cosim::enu_to_geodetic(c.east, c.north, 0.0, c.o);
        // 1e-11 deg is ~1.1 um of ground distance.
        EXPECT_NEAR(g.lat_deg, c.lat_ref, 1e-11)
            << "lat0=" << c.o.lat_deg << " e=" << c.east << " n=" << c.north;
        EXPECT_NEAR(g.lon_deg, c.lon_ref, 1e-11)
            << "lat0=" << c.o.lat_deg << " e=" << c.east << " n=" << c.north;
        EXPECT_NEAR(g.alt_m, c.alt_ref, 1e-6)
            << "lat0=" << c.o.lat_deg << " e=" << c.east << " n=" << c.north;
    }
}

// The closed-form inverse builds lat/lon from atan2(), so the result is always
// range-reduced: crossing the antimeridian can no longer emit lon > 180.
TEST(CommsTemplates, EnuToGeodeticNormalizesAcrossTheAntimeridian) {
    const GeodeticOrigin o{0.0, 179.9, 0.0};
    const auto g = vdsim::cosim::enu_to_geodetic(50000.0, 0.0, 0.0, o);
    EXPECT_NEAR(g.lon_deg, -179.650851558491468, 1e-11);   // PROJ reference
    EXPECT_GE(g.lon_deg, -180.0);
    EXPECT_LE(g.lon_deg, 180.0);
    // ... and an out-of-range datum is reduced too, rather than propagated.
    const GeodeticOrigin wrapped{0.0, 179.9 + 360.0, 0.0};
    const auto w = vdsim::cosim::enu_to_geodetic(50000.0, 0.0, 0.0, wrapped);
    EXPECT_NEAR(w.lon_deg, g.lon_deg, 1e-9);
    // Every latitude stays inside [-90, 90] even when driven over the pole.
    const GeodeticOrigin polar{89.9, 0.0, 0.0};
    for (double n : {-5.0e6, -1.0e5, 0.0, 1.0e5, 5.0e6}) {
        const auto p = vdsim::cosim::enu_to_geodetic(0.0, n, 0.0, polar);
        EXPECT_GE(p.lat_deg, -90.0);
        EXPECT_LE(p.lat_deg, 90.0);
        EXPECT_GE(p.lon_deg, -180.0);
        EXPECT_LE(p.lon_deg, 180.0);
    }
}

TEST(CommsTemplates, UtcFieldFormatsAndCarries) {
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(12 * 3600 + 35 * 60 + 19.0), "123519.00");
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(0.0), "000000.00");
    // Seconds that round up to 60.00 must carry into the minute, the minute
    // into the hour, and 23:59:59.997 must wrap the day - never "125960.00".
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(59.997), "000100.00");
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(12 * 3600 + 34 * 60 + 59.997), "123500.00");
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(12 * 3600 + 59 * 60 + 59.997), "130000.00");
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(23 * 3600 + 59 * 60 + 59.997), "000000.00");
    // Out-of-range input wraps; non-finite input yields the NMEA "null" field.
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(86400.0 + 61.25), "000101.25");
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(-1.0), "235959.00");
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(kNaN), "");
    EXPECT_EQ(vdsim::cosim::nmea_utc_field(kInf), "");
}

// Sweep the whole day at the 200 Hz tick period: no time field may ever print
// an illegal 60th second, 60th minute or 24th hour.
TEST(CommsTemplates, UtcFieldNeverPrintsAnIllegalTime) {
    for (long i = 0; i < 86400L * 200L; i += 199) {   // ~87k samples, prime stride
        const double sod = static_cast<double>(i) / 200.0;
        const std::string t = vdsim::cosim::nmea_utc_field(sod);
        ASSERT_EQ(t.size(), 9u) << sod << " -> " << t;
        const int hh = std::stoi(t.substr(0, 2));
        const int mm = std::stoi(t.substr(2, 2));
        const double ss = std::stod(t.substr(4));
        ASSERT_LT(hh, 24) << sod << " -> " << t;
        ASSERT_LT(mm, 60) << sod << " -> " << t;
        ASSERT_LT(ss, 60.0) << sod << " -> " << t;
    }
    // The exact rounding boundary, which "%05.2f" of the raw seconds got wrong.
    for (double eps : {0.0, 0.001, 0.002, 0.0025, 0.003, 0.004, 0.0049}) {
        const std::string t = vdsim::cosim::nmea_utc_field(59.995 + eps);
        EXPECT_EQ(t, "000100.00") << 59.995 + eps << " -> " << t;
    }
}

TEST(CommsTemplates, GgaZeroOffsetAtOrigin) {
    StateFields s;                      // m_gnss_x/_y default to 0 -> at the datum
    s.z = 0.0;
    const GeodeticOrigin o{48.1173, 11.5166666667, 545.4};
    const auto f = parse_checked_sentence(
        vdsim::cosim::encode_gga(s, o, 12 * 3600 + 35 * 60 + 19.0));
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[0], "GPGGA");
    EXPECT_EQ(f[1], "123519.00");
    EXPECT_EQ(f[2], "4807.0380");
    EXPECT_EQ(f[3], "N");
    EXPECT_EQ(f[4], "01131.0000");
    EXPECT_EQ(f[5], "E");
    EXPECT_EQ(f[6], "1");             // fix quality
    EXPECT_EQ(f[7], "12");            // satellites (synthetic)
    EXPECT_EQ(f[8], "0.9");           // HDOP (synthetic)
    EXPECT_EQ(f[9], "545.4");         // altitude
    EXPECT_EQ(f[10], "M");
    EXPECT_EQ(f[11], "0.0");          // geoid separation (synthetic)
    EXPECT_EQ(f[12], "M");
    EXPECT_EQ(f[13], "");             // DGPS age: not available
    EXPECT_EQ(f[14], "");             // DGPS station id: not available
}

TEST(CommsTemplates, GgaSouthernAndWesternHemisphere) {
    // Origin in the southern/western hemisphere, vehicle at the datum: the
    // sentence must report S and W with unsigned degree+minute fields.
    StateFields s;
    const GeodeticOrigin o{-33.8688, -70.6693, 570.0};
    const auto f = parse_checked_sentence(vdsim::cosim::encode_gga(s, o, 0.0));
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[1], "000000.00");
    EXPECT_EQ(f[2], "3352.1280");
    EXPECT_EQ(f[3], "S");
    EXPECT_EQ(f[4], "07040.1580");
    EXPECT_EQ(f[5], "W");
}

TEST(CommsTemplates, GgaUsesMeasuredGnssAndEnuAxes) {
    // x = ENU east, y = ENU north (core/include/vdsim/coordinate.hpp).
    // Truth x/y is deliberately set to a different place from the measured GNSS
    // to prove the sentence follows the receiver output, not the truth pose.
    StateFields s;
    s.x = -5000.0; s.y = -5000.0;
    s.m_gnss_x = kPrimeVertDegAtEq;   // ~1 deg east
    s.m_gnss_y = kMeridianDegAtEq;    // ~1 deg north
    s.z = 0.0;
    const GeodeticOrigin o{0.0, 0.0, 0.0};
    const auto f = parse_checked_sentence(vdsim::cosim::encode_gga(s, o, 0.0));
    ASSERT_EQ(f.size(), 15u);
    // PROJ reference: lat 0.999744179416095, lon 0.999898479474795.
    EXPECT_EQ(f[2], "0059.9847");
    EXPECT_EQ(f[3], "N");
    EXPECT_EQ(f[4], "00059.9939");
    EXPECT_EQ(f[5], "E");
}

// D5: an origin near the antimeridian plus an eastward offset must not emit an
// out-of-range "18020.9495,E" - it is 179 deg 39.05 min WEST.
TEST(CommsTemplates, GgaNormalizesLongitudeAcrossTheAntimeridian) {
    StateFields s;
    s.m_gnss_x = 50000.0;
    const GeodeticOrigin o{0.0, 179.9, 0.0};
    const auto f = parse_checked_sentence(vdsim::cosim::encode_gga(s, o, 0.0));
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[4], "17939.0511");
    EXPECT_EQ(f[5], "W");
    EXPECT_LT(std::stod(f[4]), 18100.0);   // ddd part never exceeds 180
}

// D3: a diverged solver must produce a well-formed "no fix" sentence, never a
// checksum-valid sentence with "nan" and embedded spaces inside its fields.
TEST(CommsTemplates, GgaReportsNoFixForDegenerateState) {
    const GeodeticOrigin o{37.5665, 126.9780, 38.0};
    struct Case { const char* name; double gx, gy, z; };
    const Case cases[] = {
        {"nan position",  kNaN,  kNaN,  kNaN},
        {"nan x only",    kNaN,  10.0,  0.0},
        {"inf x",         kInf,  0.0,   0.0},
        {"-inf y",        0.0,  -kInf,  0.0},
        {"nan altitude",  10.0,  10.0,  kNaN},
        {"1e18 offset",   1.0e18, 1.0e18, 1.0e18},
        {"1e300 offset",  1.0e300, 0.0, 0.0},
    };
    for (const Case& c : cases) {
        StateFields s;
        s.m_gnss_x = c.gx; s.m_gnss_y = c.gy; s.z = c.z;
        const std::string sentence = vdsim::cosim::encode_gga(s, o, 3600.0);
        const auto f = parse_checked_sentence(sentence);
        ASSERT_EQ(f.size(), 15u) << c.name << ": " << sentence;
        EXPECT_EQ(f[2], "") << c.name;      // latitude: not available
        EXPECT_EQ(f[3], "") << c.name;
        EXPECT_EQ(f[4], "") << c.name;      // longitude: not available
        EXPECT_EQ(f[5], "") << c.name;
        EXPECT_EQ(f[6], "0")  << c.name;    // fix quality 0 = invalid
        EXPECT_EQ(f[7], "00") << c.name;
        EXPECT_EQ(f[9], "") << c.name;      // altitude: not available
        EXPECT_EQ(f[10], "M") << c.name;
        EXPECT_EQ(f[12], "M") << c.name;
        // No field may contain whitespace, "nan" or "inf": a consumer splitting
        // on commas must never see a token it cannot parse as a number.
        for (const std::string& field : f) {
            EXPECT_EQ(field.find(' '), std::string::npos) << c.name << ": " << sentence;
            EXPECT_EQ(field.find("nan"), std::string::npos) << c.name << ": " << sentence;
            EXPECT_EQ(field.find("inf"), std::string::npos) << c.name << ": " << sentence;
        }
    }
    // A non-finite datum is rejected the same way.
    StateFields ok;
    const GeodeticOrigin bad{kNaN, 126.978, 38.0};
    const auto f = parse_checked_sentence(vdsim::cosim::encode_gga(ok, bad, 0.0));
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[6], "0");
}

// N1 regression, layer 1: the ECEF->geodetic inverse is only single-valued
// outside the ellipsoid's evolute. Near/below the geocentre the Newton step's
// atan2() x-argument flips sign and returns a second/third-quadrant angle -
// a "latitude" of |lat| > 90 deg. That must be reported as failure, not
// returned as an angle.
TEST(CommsTemplates, EnuToGeodeticFailsInsideTheEarthInsteadOfReturningABadAngle) {
    const GeodeticOrigin o{37.5665, 126.9780, 38.0};
    // The exact reproduction: a pure vertical divergence 6350 km down, which is
    // still inside the kMaxEnuOffsetM = 1e8 m coarse guard and used to yield
    // lat_deg = -163.551823.
    const auto bad = vdsim::cosim::enu_to_geodetic(0.0, 0.0, -6.35e6, o);
    EXPECT_FALSE(bad.ok);
    EXPECT_TRUE(std::isnan(bad.lat_deg));
    EXPECT_TRUE(std::isnan(bad.lon_deg));
    EXPECT_TRUE(std::isnan(bad.alt_m));
    // The whole "inside the Earth" band, at 1 m steps: every result is either a
    // clean failure or a latitude that actually exists.
    for (double z = -6.5e6; z <= -6.2e6; z += 1.0) {
        const auto g = vdsim::cosim::enu_to_geodetic(0.0, 0.0, z, o);
        if (!g.ok) continue;
        ASSERT_LE(std::fabs(g.lat_deg), 90.0) << "z=" << z;
        ASSERT_LE(std::fabs(g.lon_deg), 180.0) << "z=" << z;
    }
    // Non-finite inputs and a non-finite datum fail rather than propagating.
    EXPECT_FALSE(vdsim::cosim::enu_to_geodetic(kNaN, 0.0, 0.0, o).ok);
    EXPECT_FALSE(vdsim::cosim::enu_to_geodetic(0.0, kInf, 0.0, o).ok);
    EXPECT_FALSE(vdsim::cosim::enu_to_geodetic(0.0, 0.0, 0.0,
                                               GeodeticOrigin{kNaN, 0.0, 0.0}).ok);
    // A default-constructed result is "no fix", not a fix at Null Island.
    EXPECT_FALSE(vdsim::cosim::Geodetic{}.ok);
    // Ordinary vehicle states keep reporting success.
    for (double z : {-1000.0, -10.0, 0.0, 10.0, 1000.0, 1.0e4}) {
        const auto g = vdsim::cosim::enu_to_geodetic(1234.0, -5678.0, z, o);
        EXPECT_TRUE(g.ok) << "z=" << z;
    }
}

// N1 regression, layer 2: the NMEA angle field bound is per-field. Checking
// everything against 180 let a latitude of 163.55 deg through, which also
// widened the field to ten characters where GGA specifies nine (ddmm.mmmm).
TEST(CommsTemplates, DegMinUsesThePerFieldRangeBound) {
    EXPECT_DOUBLE_EQ(vdsim::cosim::nmea_deg_max(2), 90.0);
    EXPECT_DOUBLE_EQ(vdsim::cosim::nmea_deg_max(3), 180.0);
    // The reproduced value: rejected as a latitude, legal as a longitude.
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(-163.551823, 2), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(-163.551823, 3), "16333.1094");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(90.0001, 2), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(-90.0001, 2), "");
    EXPECT_EQ(vdsim::cosim::nmea_deg_min(90.0, 2), "9000.0000");   // inclusive
    // A latitude field is always ddmm.mmmm, a longitude field dddmm.mmmm.
    for (double d = -90.0; d <= 90.0; d += 0.37)
        ASSERT_EQ(vdsim::cosim::nmea_deg_min(d, 2).size(), 9u) << d;
    for (double d = -180.0; d <= 180.0; d += 0.73)
        ASSERT_EQ(vdsim::cosim::nmea_deg_min(d, 3).size(), 10u) << d;
}

// N1 regression, layer 3: a state the projection cannot invert must take the
// same D3 "no fix" path as NaN, never a checksum-valid quality-1 sentence with
// an impossible latitude.
TEST(CommsTemplates, GgaNeverEmitsAQualityOneFixWithAnOutOfRangeField) {
    const GeodeticOrigin o{37.5665, 126.9780, 38.0};
    // Asserts a sentence is either a well-formed fix or the "no fix" form.
    auto check = [](const std::vector<std::string>& f, double z) {
        ASSERT_EQ(f.size(), 15u) << "z=" << z;
        if (f[6] != "1") {                       // "no fix": D3's degradation
            EXPECT_EQ(f[2], "")     << "z=" << z;
            EXPECT_EQ(f[3], "")     << "z=" << z;
            EXPECT_EQ(f[4], "")     << "z=" << z;
            EXPECT_EQ(f[5], "")     << "z=" << z;
            EXPECT_EQ(f[6], "0")    << "z=" << z;
            EXPECT_EQ(f[7], "00")   << "z=" << z;
            EXPECT_EQ(f[8], "99.9") << "z=" << z;
            EXPECT_EQ(f[9], "")     << "z=" << z;
            return;
        }
        ASSERT_EQ(f[2].size(), 9u) << "z=" << z << " lat field " << f[2];
        ASSERT_EQ(f[4].size(), 10u) << "z=" << z << " lon field " << f[4];
        EXPECT_TRUE(f[3] == "N" || f[3] == "S") << "z=" << z;
        EXPECT_TRUE(f[5] == "E" || f[5] == "W") << "z=" << z;
        EXPECT_LE(std::stod(f[2]), 9000.0) << "z=" << z;    // |lat| <= 90 deg
        EXPECT_LE(std::stod(f[4]), 18000.0) << "z=" << z;   // |lon| <= 180 deg
        EXPECT_LT(std::stod(f[2].substr(2)), 60.0) << "z=" << z;
        EXPECT_LT(std::stod(f[4].substr(3)), 60.0) << "z=" << z;
    };
    // The exact reproduction, which emitted
    // "$GPGGA,010000.00,16333.1094,S,12658.6800,E,1,12,0.9,-6402744.3,M,0.0,M,,*5D".
    StateFields s;
    s.z = -6.35e6;
    const std::string sentence = vdsim::cosim::encode_gga(s, o, 3600.0);
    const auto f = parse_checked_sentence(sentence);
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[6], "0") << sentence;
    EXPECT_EQ(f[2], "") << sentence;
    check(f, s.z);
    // The measured reachable bands, at 1 m steps, plus the surrounding region.
    const double bands[][3] = {{-6361743.0, -6360052.0,  1.0},
                               {-6356268.0, -6337299.0,  1.0},
                               {-6334874.0, -6334540.0,  1.0},
                               {-6500000.0, -6200000.0, 11.0}};   // surrounding region
    for (const auto& b : bands) {
        for (double z = b[0]; z <= b[1]; z += b[2]) {
            StateFields t;
            t.z = z;
            check(parse_checked_sentence(vdsim::cosim::encode_gga(t, o, 3600.0)), z);
        }
    }
    // ... and a coarse pass over the whole |z| range the coarse guard accepts,
    // with horizontal offsets, so no other band can hide.
    for (double h : {0.0, 1.0e3, 1.0e5, 1.0e7}) {
        for (double z = -1.0e8; z <= 1.0e8; z += 5.0e4) {
            StateFields t;
            t.m_gnss_x = h; t.m_gnss_y = -h; t.z = z;
            check(parse_checked_sentence(vdsim::cosim::encode_gga(t, o, 3600.0)), z);
        }
    }
}

// An altitude too large to format must blank the field, not overflow it, while
// the horizontal fix (which is still meaningful) is kept.
TEST(CommsTemplates, GgaBlanksAnUnprintableAltitude) {
    StateFields s;
    s.m_gnss_x = 100.0; s.m_gnss_y = 100.0;
    s.z = 5.0e7;                       // above kMaxGgaAltitudeM, still << kMaxEnuOffsetM
    const GeodeticOrigin o{37.5665, 126.9780, 38.0};
    const auto f = parse_checked_sentence(vdsim::cosim::encode_gga(s, o, 0.0));
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[6], "1");              // still a valid fix
    EXPECT_NE(f[2], "");
    EXPECT_EQ(f[9], "");               // altitude: not available
    EXPECT_EQ(f[10], "M");
}

// D4 at the encoder level: no encoded sentence may carry an illegal UTC field.
TEST(CommsTemplates, GgaNeverEncodesAnIllegalUtcField) {
    StateFields s;
    const GeodeticOrigin o{37.5665, 126.9780, 38.0};
    for (double base : {0.0, 45299.0, 46799.0, 86399.0}) {
        for (double frac : {0.99, 0.995, 0.996, 0.997, 0.999, 0.9999}) {
            const auto f = parse_checked_sentence(
                vdsim::cosim::encode_gga(s, o, base + frac));
            ASSERT_EQ(f.size(), 15u);
            ASSERT_EQ(f[1].size(), 9u) << base + frac << " -> " << f[1];
            EXPECT_LT(std::stoi(f[1].substr(0, 2)), 24) << f[1];
            EXPECT_LT(std::stoi(f[1].substr(2, 2)), 60) << f[1];
            EXPECT_LT(std::stod(f[1].substr(4)), 60.0) << f[1];
        }
    }
}

TEST(CommsTemplates, GgaFitsTheServerTxBuffer) {
    StateFields s;
    s.m_gnss_x = 1.0e5; s.m_gnss_y = -1.0e5; s.z = -1234.5;
    uint8_t buf[vdsim::cosim::kTxBufBytes];
    const GeodeticOrigin o{-45.0, -70.0, 0.0};
    const int n = vdsim::cosim::encode_state_nmea_gga(buf, sizeof(buf), s, o, 86399.99);
    ASSERT_GT(n, 0);
    EXPECT_LT(n, static_cast<int>(sizeof(buf)));
    EXPECT_EQ(static_cast<char>(buf[0]), '$');
    EXPECT_EQ(static_cast<char>(buf[n - 2]), '\r');
    EXPECT_EQ(static_cast<char>(buf[n - 1]), '\n');
    // Too small a buffer is reported, never silently truncated.
    uint8_t tiny[8];
    EXPECT_EQ(vdsim::cosim::encode_state_nmea_gga(tiny, sizeof(tiny), s, o, 0.0), -1);
}

TEST(CommsTemplates, UtcSecondsOfDayInRange) {
    const double sod = vdsim::cosim::utc_seconds_of_day_now();
    EXPECT_GE(sod, 0.0);
    EXPECT_LT(sod, 86400.0);
}

TEST(CommsTemplates, JsonNumberFormatsOrEmitsNull) {
    EXPECT_EQ(vdsim::cosim::json_number(1.25, 2), "1.25");
    EXPECT_EQ(vdsim::cosim::json_number(-0.5, 4), "-0.5000");
    EXPECT_EQ(vdsim::cosim::json_number(4500.0, 0), "4500");
    // RFC 8259 has no NaN/Infinity literal; "nan"/"inf" would break every parser.
    EXPECT_EQ(vdsim::cosim::json_number(kNaN, 2), "null");
    EXPECT_EQ(vdsim::cosim::json_number(kInf, 2), "null");
    EXPECT_EQ(vdsim::cosim::json_number(-kInf, 4), "null");
    // Huge-but-finite values are printed in full, never truncated to a wrong
    // number by the fixed scratch buffer.
    const std::string big = vdsim::cosim::json_number(1.0e300, 2);
    EXPECT_EQ(big.size(), 304u) << big.substr(0, 32);   // 301 digits + "." + 2
    EXPECT_EQ(big.substr(0, 1), "1");
    EXPECT_EQ(big.substr(big.size() - 3), ".00");
}

TEST(CommsTemplates, JsonNumberArrayJoinsWithCommas) {
    const double v[4] = {1.0, kNaN, -2.5, kInf};
    EXPECT_EQ(vdsim::cosim::json_number_array(v, 4, 1), "1.0,null,-2.5,null");
    EXPECT_EQ(vdsim::cosim::json_number_array(v, 1, 0), "1");
    EXPECT_EQ(vdsim::cosim::json_number_array(v, 0, 2), "");
}

TEST(CommsTemplates, JsonEncodesWellFormedObject) {
    StateFields s;
    s.vehicle_id = 7; s.timestamp = 1.25;
    s.x = 45.6; s.y = -3.2; s.yaw = 0.04;
    s.vx = 15.1; s.vy = 0.2; s.yaw_rate = 0.01;
    s.ax = 0.3; s.ay = 1.2;
    for (int i = 0; i < 4; ++i) { s.Fz[i] = 4500.0; s.slip_angle[i] = 0.01; s.slip_ratio[i] = 0.02; }
    uint8_t buf[vdsim::cosim::kTxBufBytes];
    const int n = vdsim::cosim::encode_state_json(buf, sizeof(buf), s);
    ASSERT_GT(n, 0);
    const std::string j(reinterpret_cast<const char*>(buf), n);
    EXPECT_EQ(j.front(), '{');
    EXPECT_EQ(j.back(), '}');
    EXPECT_NE(j.find("\"id\":7"), std::string::npos) << j;
    EXPECT_NE(j.find("\"t\":1.250"), std::string::npos) << j;
    EXPECT_NE(j.find("\"Fz\":[4500,4500,4500,4500]"), std::string::npos) << j;
    // Balanced braces/brackets and no embedded NUL: a truncated body would fail.
    int braces = 0, brackets = 0;
    for (char c : j) {
        if (c == '{') ++braces;
        else if (c == '}') --braces;
        else if (c == '[') ++brackets;
        else if (c == ']') --brackets;
        EXPECT_NE(c, '\0');
    }
    EXPECT_EQ(braces, 0);
    EXPECT_EQ(brackets, 0);
}

// D2: a diverged solver used to put "nan"/"inf" tokens on the wire in a ~190 B
// frame that sailed under the truncation guard, so json.loads and JSON.parse
// both rejected it. Every non-finite field must now be the literal `null`.
TEST(CommsTemplates, JsonEmitsNullForNonFiniteState) {
    StateFields s;
    s.vehicle_id = 3; s.timestamp = 1.5;
    s.x = kNaN; s.y = -kInf; s.yaw = kInf;
    s.vx = kNaN; s.vy = 1.0; s.yaw_rate = kNaN;
    s.ax = 0.5; s.ay = kNaN;
    for (int i = 0; i < 4; ++i) { s.Fz[i] = kNaN; s.slip_angle[i] = kInf; s.slip_ratio[i] = 0.02; }
    uint8_t buf[vdsim::cosim::kTxBufBytes];
    const int n = vdsim::cosim::encode_state_json(buf, sizeof(buf), s);
    ASSERT_GT(n, 0);
    const std::string j(reinterpret_cast<const char*>(buf), n);
    EXPECT_EQ(j.find("nan"), std::string::npos) << j;
    EXPECT_EQ(j.find("inf"), std::string::npos) << j;
    EXPECT_EQ(j.find("NaN"), std::string::npos) << j;
    EXPECT_NE(j.find("\"x\":null"), std::string::npos) << j;
    EXPECT_NE(j.find("\"y\":null"), std::string::npos) << j;
    EXPECT_NE(j.find("\"yaw\":null"), std::string::npos) << j;
    EXPECT_NE(j.find("\"vy\":1.00"), std::string::npos) << j;    // good fields survive
    EXPECT_NE(j.find("\"ax\":0.50"), std::string::npos) << j;
    EXPECT_NE(j.find("\"Fz\":[null,null,null,null]"), std::string::npos) << j;
    EXPECT_NE(j.find("\"alpha\":[null,null,null,null]"), std::string::npos) << j;
    EXPECT_NE(j.find("\"kappa\":[0.0200,0.0200,0.0200,0.0200]"), std::string::npos) << j;
}

TEST(CommsTemplates, JsonReportsTruncationInsteadOfEmittingInvalidJson) {
    StateFields s;
    uint8_t small[32];
    EXPECT_EQ(vdsim::cosim::encode_state_json(small, sizeof(small), s), -1);
    // Exactly-fitting buffer: the encoder needs one extra byte for snprintf's NUL.
    uint8_t buf[vdsim::cosim::kTxBufBytes];
    const int n = vdsim::cosim::encode_state_json(buf, sizeof(buf), s);
    ASSERT_GT(n, 0);
    EXPECT_EQ(vdsim::cosim::encode_state_json(buf, static_cast<size_t>(n), s), -1);
    EXPECT_EQ(vdsim::cosim::encode_state_json(buf, static_cast<size_t>(n) + 1, s), n);
    // A finite value too large to fit overflows the buffer and is reported as a
    // failure rather than emitted half-written.
    StateFields huge;
    huge.x = 1.0e300; huge.y = -1.0e300;
    EXPECT_EQ(vdsim::cosim::encode_state_json(buf, sizeof(buf), huge), -1);
}
