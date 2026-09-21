// Unit tests for the receive-side JSON state and NMEA 0183 GGA parsers.
#include <gtest/gtest.h>

#include "comms_parsers.hpp"

#include <cmath>
#include <string>

using vdsim::cosim::GgaFix;
using vdsim::cosim::GeodeticOrigin;
using vdsim::cosim::ParseStatus;
using vdsim::cosim::StateFields;

namespace {

ParseStatus parse_json_text(const std::string& text, StateFields& state) {
    return vdsim::cosim::parse_state_json(
        reinterpret_cast<const uint8_t*>(text.data()), text.size(), state);
}

ParseStatus parse_gga_text(const std::string& text, GgaFix& fix) {
    return vdsim::cosim::parse_nmea_gga(
        reinterpret_cast<const uint8_t*>(text.data()), text.size(), fix);
}

}  // namespace

TEST(CommsJsonParser, RoundTripsTransmittedState) {
    StateFields input;
    input.vehicle_id = 17;
    input.timestamp = 12.375;
    input.x = 45.25; input.y = -3.5; input.yaw = 0.125;
    input.vx = 14.75; input.vy = -0.25; input.yaw_rate = 0.03125;
    input.ax = 1.25; input.ay = -2.5;
    for (int i = 0; i < 4; ++i) {
        input.Fz[i] = 4000.0 + i;
        input.slip_angle[i] = 0.01 * i;
        input.slip_ratio[i] = -0.02 * i;
    }
    uint8_t buf[vdsim::cosim::kTxBufBytes];
    const int n = vdsim::cosim::encode_state_json(buf, sizeof(buf), input);
    ASSERT_GT(n, 0);

    StateFields output;
    const ParseStatus status = vdsim::cosim::parse_state_json(buf, n, output);
    ASSERT_TRUE(status.ok) << status.error;
    EXPECT_EQ(output.vehicle_id, 17u);
    EXPECT_DOUBLE_EQ(output.timestamp, 12.375);
    EXPECT_DOUBLE_EQ(output.x, 45.25);
    EXPECT_DOUBLE_EQ(output.y, -3.5);
    EXPECT_DOUBLE_EQ(output.yaw, 0.125);
    EXPECT_DOUBLE_EQ(output.vx, 14.75);
    EXPECT_DOUBLE_EQ(output.vy, -0.25);
    EXPECT_NEAR(output.yaw_rate, 0.0312, 1e-12);
    EXPECT_DOUBLE_EQ(output.ax, 1.25);
    EXPECT_DOUBLE_EQ(output.ay, -2.5);
    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.Fz[i], 4000.0 + i);
        EXPECT_DOUBLE_EQ(output.slip_angle[i], 0.01 * i);
        EXPECT_DOUBLE_EQ(output.slip_ratio[i], -0.02 * i);
    }
}

TEST(CommsJsonParser, AcceptsWhitespaceOrderAndUnknownField) {
    const std::string text =
        "{ \"extra\":{\"future\":[true,false,null]}, \"kappa\":[0,0,0,0],"
        "\"alpha\":[0,0,0,0],\"Fz\":[1,2,3,4],\"ay\":0,\"ax\":0,"
        "\"r\":0,\"vy\":0,\"vx\":1,\"yaw\":0,\"y\":2,\"x\":1,"
        "\"t\":3,\"id\":4 }";
    StateFields state;
    const ParseStatus status = parse_json_text(text, state);
    ASSERT_TRUE(status.ok) << status.error;
    EXPECT_EQ(state.vehicle_id, 4u);
    EXPECT_DOUBLE_EQ(state.Fz[3], 4.0);
}

TEST(CommsJsonParser, RestoresNullAsNan) {
    const std::string text =
        "{\"id\":0,\"t\":null,\"x\":null,\"y\":0,\"yaw\":0,"
        "\"vx\":0,\"vy\":0,\"r\":0,\"ax\":0,\"ay\":0,"
        "\"Fz\":[null,1,2,3],\"alpha\":[0,0,0,0],\"kappa\":[0,0,0,0]}";
    StateFields state;
    const ParseStatus status = parse_json_text(text, state);
    ASSERT_TRUE(status.ok) << status.error;
    EXPECT_TRUE(std::isnan(state.timestamp));
    EXPECT_TRUE(std::isnan(state.x));
    EXPECT_TRUE(std::isnan(state.Fz[0]));
}

TEST(CommsJsonParser, RejectsMissingRequiredField) {
    const std::string text =
        "{\"id\":0,\"t\":0,\"x\":0,\"y\":0,\"vx\":0,\"vy\":0,"
        "\"r\":0,\"ax\":0,\"ay\":0,\"Fz\":[0,0,0,0],"
        "\"alpha\":[0,0,0,0],\"kappa\":[0,0,0,0]}";
    StateFields state;
    const ParseStatus status = parse_json_text(text, state);
    EXPECT_FALSE(status.ok);
    EXPECT_EQ(status.error, "json: missing one or more required state fields");
}

TEST(CommsJsonParser, RejectsWrongWheelArrayLength) {
    const std::string text =
        "{\"id\":0,\"t\":0,\"x\":0,\"y\":0,\"yaw\":0,\"vx\":0,"
        "\"vy\":0,\"r\":0,\"ax\":0,\"ay\":0,\"Fz\":[0,0,0],"
        "\"alpha\":[0,0,0,0],\"kappa\":[0,0,0,0]}";
    StateFields state;
    const ParseStatus status = parse_json_text(text, state);
    EXPECT_FALSE(status.ok);
    EXPECT_NE(status.error.find("exactly four"), std::string::npos) << status.error;
}

TEST(CommsGgaParser, ParsesReferenceSentence) {
    const std::string sentence =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    GgaFix fix;
    const ParseStatus status = parse_gga_text(sentence, fix);
    ASSERT_TRUE(status.ok) << status.error;
    EXPECT_TRUE(fix.has_fix());
    EXPECT_DOUBLE_EQ(fix.utc_sod, 12 * 3600.0 + 35 * 60.0 + 19.0);
    EXPECT_NEAR(fix.lat_deg, 48.1173, 1e-10);
    EXPECT_NEAR(fix.lon_deg, 11.5166666667, 1e-10);
    EXPECT_DOUBLE_EQ(fix.alt_m, 545.4);
    EXPECT_EQ(fix.satellites, 8);
}

TEST(CommsGgaParser, RoundTripsTransmittedFix) {
    StateFields state;
    state.m_gnss_x = 12.5;
    state.m_gnss_y = -7.25;
    state.z = 3.0;
    const GeodeticOrigin origin{37.5665, 126.9780, 38.0};
    uint8_t buf[vdsim::cosim::kTxBufBytes];
    const int n = vdsim::cosim::encode_state_nmea_gga(buf, sizeof(buf), state, origin, 43210.5);
    ASSERT_GT(n, 0);
    GgaFix fix;
    const ParseStatus status = vdsim::cosim::parse_nmea_gga(buf, n, fix);
    ASSERT_TRUE(status.ok) << status.error;
    EXPECT_TRUE(fix.has_fix());
    EXPECT_EQ(fix.fix_quality, vdsim::cosim::kGgaFixQuality);
    EXPECT_EQ(fix.satellites, vdsim::cosim::kGgaSatellites);
    EXPECT_NEAR(fix.utc_sod, 43210.5, 1e-9);
}

TEST(CommsGgaParser, ParsesNoFixSentence) {
    StateFields state;
    state.m_gnss_x = std::numeric_limits<double>::quiet_NaN();
    uint8_t buf[vdsim::cosim::kTxBufBytes];
    const int n = vdsim::cosim::encode_state_nmea_gga(
        buf, sizeof(buf), state, GeodeticOrigin{}, 1.0);
    ASSERT_GT(n, 0);
    GgaFix fix;
    const ParseStatus status = vdsim::cosim::parse_nmea_gga(buf, n, fix);
    ASSERT_TRUE(status.ok) << status.error;
    EXPECT_FALSE(fix.has_fix());
    EXPECT_TRUE(std::isnan(fix.lat_deg));
    EXPECT_TRUE(std::isnan(fix.lon_deg));
}

TEST(CommsGgaParser, RejectsChecksumMismatch) {
    const std::string sentence =
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n";
    GgaFix fix;
    const ParseStatus status = parse_gga_text(sentence, fix);
    EXPECT_FALSE(status.ok);
    EXPECT_EQ(status.error, "nmea_gga: checksum mismatch (expected 47, got 00)");
}

TEST(CommsGgaParser, RejectsOutOfRangeCoordinate) {
    const std::string body =
        "GPGGA,123519,9100.000,N,01131.000,E,1,08,0.9,1.0,M,0.0,M,,";
    const std::string sentence = vdsim::cosim::nmea_sentence({
        "GPGGA", "123519", "9100.000", "N", "01131.000", "E", "1", "08",
        "0.9", "1.0", "M", "0.0", "M", "", ""});
    ASSERT_FALSE(body.empty());
    GgaFix fix;
    const ParseStatus status = parse_gga_text(sentence, fix);
    EXPECT_FALSE(status.ok);
    EXPECT_EQ(status.error, "nmea_gga: invalid latitude field");
}
