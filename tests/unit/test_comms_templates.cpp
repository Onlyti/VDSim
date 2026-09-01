// Unit tests for the text TX templates (cosim/comms_templates.hpp):
// template canonicalisation, NMEA 0183 checksum/framing, ENU->geodetic
// projection, GGA field layout and the JSON state encoder.
#include <gtest/gtest.h>

#include "comms_templates.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

using vdsim::cosim::GeodeticOrigin;
using vdsim::cosim::StateFields;

namespace {

// WGS84 radii of curvature at the equator, used to turn a metre offset into a
// whole number of degrees for the formatting checks below.
constexpr double kMeridianDegAtEq = 110574.2727;   // 1 deg north at lat0 = 0 [m]
constexpr double kPrimeVertDegAtEq = 111319.4908;  // 1 deg east  at lat0 = 0 [m]

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

TEST(CommsTemplates, EnuToGeodeticZeroOffsetIsExactlyTheOrigin) {
    const GeodeticOrigin o{37.5665, 126.9780, 38.0};
    const auto g = vdsim::cosim::enu_to_geodetic(0.0, 0.0, 0.0, o);
    EXPECT_DOUBLE_EQ(g.lat_deg, o.lat_deg);
    EXPECT_DOUBLE_EQ(g.lon_deg, o.lon_deg);
    EXPECT_DOUBLE_EQ(g.alt_m, o.alt_m);
}

TEST(CommsTemplates, EnuToGeodeticKnownOffsetAtEquator) {
    const GeodeticOrigin o{0.0, 0.0, 0.0};
    const auto g = vdsim::cosim::enu_to_geodetic(kPrimeVertDegAtEq,
                                                 kMeridianDegAtEq, 12.5, o);
    EXPECT_NEAR(g.lat_deg, 1.0, 1e-6);
    EXPECT_NEAR(g.lon_deg, 1.0, 1e-6);
    EXPECT_DOUBLE_EQ(g.alt_m, 12.5);
    // Sign convention: +east raises longitude, +north raises latitude.
    const auto neg = vdsim::cosim::enu_to_geodetic(-kPrimeVertDegAtEq,
                                                   -kMeridianDegAtEq, 0.0, o);
    EXPECT_NEAR(neg.lat_deg, -1.0, 1e-6);
    EXPECT_NEAR(neg.lon_deg, -1.0, 1e-6);
}

TEST(CommsTemplates, GgaZeroOffsetAtOrigin) {
    StateFields s;                      // m_gnss_x/_y default to 0 -> at the datum
    s.z = 0.0;
    const GeodeticOrigin o{48.1173, 11.5166666667, 545.4};
    const std::string sentence = vdsim::cosim::encode_gga(s, o, 12 * 3600 + 35 * 60 + 19.0);
    const auto [body, hex] = split_sentence(sentence.substr(0, sentence.size() - 2));
    const auto f = fields_of(body);
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
    char expect[4];
    std::snprintf(expect, sizeof(expect), "%02X", vdsim::cosim::nmea_checksum(body));
    EXPECT_EQ(hex, std::string(expect));
}

TEST(CommsTemplates, GgaSouthernAndWesternHemisphere) {
    // Origin one degree south / one degree west of Null Island, vehicle at the
    // datum: the sentence must report S and W with unsigned degree+minute fields.
    StateFields s;
    const GeodeticOrigin o{-33.8688, -70.6693, 570.0};
    const std::string sentence = vdsim::cosim::encode_gga(s, o, 0.0);
    const auto [body, hex] = split_sentence(sentence.substr(0, sentence.size() - 2));
    const auto f = fields_of(body);
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[1], "000000.00");
    EXPECT_EQ(f[2], "3352.1280");
    EXPECT_EQ(f[3], "S");
    EXPECT_EQ(f[4], "07040.1580");
    EXPECT_EQ(f[5], "W");
    char expect[4];
    std::snprintf(expect, sizeof(expect), "%02X", vdsim::cosim::nmea_checksum(body));
    EXPECT_EQ(hex, std::string(expect));
}

TEST(CommsTemplates, GgaUsesMeasuredGnssAndEnuAxes) {
    // x = ENU east, y = ENU north (core/include/vdsim/coordinate.hpp).
    // Truth x/y is deliberately set to a different place from the measured GNSS
    // to prove the sentence follows the receiver output, not the truth pose.
    StateFields s;
    s.x = -5000.0; s.y = -5000.0;
    s.m_gnss_x = kPrimeVertDegAtEq;   // one degree east
    s.m_gnss_y = kMeridianDegAtEq;    // one degree north
    s.z = 0.0;
    const GeodeticOrigin o{0.0, 0.0, 0.0};
    const auto f = fields_of(split_sentence(vdsim::cosim::encode_gga(s, o, 0.0)).first);
    ASSERT_EQ(f.size(), 15u);
    EXPECT_EQ(f[2], "0100.0000");
    EXPECT_EQ(f[3], "N");
    EXPECT_EQ(f[4], "00100.0000");
    EXPECT_EQ(f[5], "E");
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
}
