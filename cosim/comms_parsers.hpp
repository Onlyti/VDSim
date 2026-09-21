// Receive-side parsers for the text formats emitted by comms_templates.hpp.
//
// This file deliberately does not alter the TX encoders.  It accepts their
// wire formats and returns typed values for applications that receive VDSim
// telemetry over UDP.
#pragma once

#include "comms_templates.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace vdsim::cosim {

/// @brief Result shared by the JSON and NMEA receive parsers.
struct ParseStatus {
    bool ok {false};
    std::string error;

    explicit operator bool() const { return ok; }

    static ParseStatus success() { return {true, {}}; }
    static ParseStatus failure(std::string message) {
        return {false, std::move(message)};
    }
};

/// @brief Decoded fields from an NMEA 0183 GGA sentence.
struct GgaFix {
    double utc_sod {std::numeric_limits<double>::quiet_NaN()};
    double lat_deg {std::numeric_limits<double>::quiet_NaN()};
    double lon_deg {std::numeric_limits<double>::quiet_NaN()};
    double alt_m {std::numeric_limits<double>::quiet_NaN()};
    double geoid_sep_m {std::numeric_limits<double>::quiet_NaN()};
    double hdop {std::numeric_limits<double>::quiet_NaN()};
    int fix_quality {0};
    int satellites {0};

    bool has_fix() const { return fix_quality > 0; }
};

namespace detail {

class JsonCursor {
public:
    explicit JsonCursor(std::string_view input) : input_(input) {}

    size_t position() const { return pos_; }
    bool at_end() {
        whitespace();
        return pos_ == input_.size();
    }
    void whitespace() {
        while (pos_ < input_.size()) {
            const char c = input_[pos_];
            if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
            ++pos_;
        }
    }
    bool take(char expected) {
        whitespace();
        if (pos_ >= input_.size() || input_[pos_] != expected) return false;
        ++pos_;
        return true;
    }
    bool literal(std::string_view value) {
        whitespace();
        if (input_.substr(pos_, value.size()) != value) return false;
        pos_ += value.size();
        return true;
    }
    bool string(std::string& out) {
        whitespace();
        if (pos_ >= input_.size() || input_[pos_] != '"') return false;
        ++pos_;
        out.clear();
        while (pos_ < input_.size()) {
            const unsigned char c = static_cast<unsigned char>(input_[pos_++]);
            if (c == '"') return true;
            if (c < 0x20) return false;
            if (c != '\\') {
                out.push_back(static_cast<char>(c));
                continue;
            }
            if (pos_ >= input_.size()) return false;
            const char esc = input_[pos_++];
            switch (esc) {
                case '"': out.push_back('"'); break;
                case '\\': out.push_back('\\'); break;
                case '/': out.push_back('/'); break;
                case 'b': out.push_back('\b'); break;
                case 'f': out.push_back('\f'); break;
                case 'n': out.push_back('\n'); break;
                case 'r': out.push_back('\r'); break;
                case 't': out.push_back('\t'); break;
                default: return false;
            }
        }
        return false;
    }
    bool number(double& value, bool& is_null) {
        whitespace();
        if (literal("null")) {
            value = std::numeric_limits<double>::quiet_NaN();
            is_null = true;
            return true;
        }
        is_null = false;
        const size_t begin = pos_;
        if (pos_ < input_.size() && input_[pos_] == '-') ++pos_;
        if (pos_ >= input_.size()) { pos_ = begin; return false; }
        if (input_[pos_] == '0') {
            ++pos_;
        } else if (input_[pos_] >= '1' && input_[pos_] <= '9') {
            while (pos_ < input_.size() && input_[pos_] >= '0' && input_[pos_] <= '9') ++pos_;
        } else {
            pos_ = begin;
            return false;
        }
        if (pos_ < input_.size() && input_[pos_] == '.') {
            ++pos_;
            const size_t digits = pos_;
            while (pos_ < input_.size() && input_[pos_] >= '0' && input_[pos_] <= '9') ++pos_;
            if (digits == pos_) { pos_ = begin; return false; }
        }
        if (pos_ < input_.size() && (input_[pos_] == 'e' || input_[pos_] == 'E')) {
            ++pos_;
            if (pos_ < input_.size() && (input_[pos_] == '+' || input_[pos_] == '-')) ++pos_;
            const size_t digits = pos_;
            while (pos_ < input_.size() && input_[pos_] >= '0' && input_[pos_] <= '9') ++pos_;
            if (digits == pos_) { pos_ = begin; return false; }
        }
        const std::string token(input_.substr(begin, pos_ - begin));
        char* end = nullptr;
        errno = 0;
        value = std::strtod(token.c_str(), &end);
        if (errno == ERANGE || end != token.c_str() + token.size() || !std::isfinite(value)) {
            pos_ = begin;
            return false;
        }
        return true;
    }
    bool array4(double* out) {
        if (!take('[')) return false;
        for (size_t i = 0; i < 4; ++i) {
            bool is_null = false;
            if (!number(out[i], is_null)) return false;
            if (i != 3 && !take(',')) return false;
        }
        return take(']');
    }
    bool skip_value(int depth = 0) {
        if (depth > 8) return false;
        whitespace();
        if (pos_ >= input_.size()) return false;
        if (input_[pos_] == '"') {
            std::string ignored;
            return string(ignored);
        }
        if (input_[pos_] == '{') {
            ++pos_;
            whitespace();
            if (take('}')) return true;
            while (true) {
                std::string key;
                if (!string(key) || !take(':') || !skip_value(depth + 1)) return false;
                if (take('}')) return true;
                if (!take(',')) return false;
            }
        }
        if (input_[pos_] == '[') {
            ++pos_;
            whitespace();
            if (take(']')) return true;
            while (true) {
                if (!skip_value(depth + 1)) return false;
                if (take(']')) return true;
                if (!take(',')) return false;
            }
        }
        if (literal("true") || literal("false") || literal("null")) return true;
        double ignored = 0.0;
        bool is_null = false;
        return number(ignored, is_null) && !is_null;
    }

private:
    std::string_view input_;
    size_t pos_ {0};
};

inline ParseStatus json_error(const JsonCursor& cursor, const std::string& message) {
    return ParseStatus::failure("json: " + message + " at byte " +
                                std::to_string(cursor.position()));
}

inline bool parse_int(const std::string& text, int& value) {
    if (text.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (errno == ERANGE || end != text.c_str() + text.size() ||
        parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max())
        return false;
    value = static_cast<int>(parsed);
    return true;
}

inline bool parse_decimal(const std::string& text, double& value) {
    if (text.empty()) return false;
    char* end = nullptr;
    errno = 0;
    value = std::strtod(text.c_str(), &end);
    return errno != ERANGE && end == text.c_str() + text.size() && std::isfinite(value);
}

inline bool parse_hex(char c, uint8_t& value) {
    if (c >= '0' && c <= '9') value = static_cast<uint8_t>(c - '0');
    else if (c >= 'A' && c <= 'F') value = static_cast<uint8_t>(10 + c - 'A');
    else if (c >= 'a' && c <= 'f') value = static_cast<uint8_t>(10 + c - 'a');
    else return false;
    return true;
}

inline std::vector<std::string> comma_fields(std::string_view body) {
    std::vector<std::string> fields;
    size_t begin = 0;
    while (true) {
        const size_t comma = body.find(',', begin);
        if (comma == std::string_view::npos) {
            fields.emplace_back(body.substr(begin));
            break;
        }
        fields.emplace_back(body.substr(begin, comma - begin));
        begin = comma + 1;
    }
    return fields;
}

inline bool parse_utc(const std::string& text, double& utc_sod) {
    if (text.size() < 6) return false;
    for (size_t i = 0; i < 6; ++i)
        if (text[i] < '0' || text[i] > '9') return false;
    const int hh = (text[0] - '0') * 10 + text[1] - '0';
    const int mm = (text[2] - '0') * 10 + text[3] - '0';
    double ss = 0.0;
    if (!parse_decimal(text.substr(4), ss)) return false;
    if (hh > 23 || mm > 59 || ss < 0.0 || ss >= 60.0) return false;
    utc_sod = hh * 3600.0 + mm * 60.0 + ss;
    return true;
}

inline bool parse_deg_min(const std::string& text, size_t deg_digits,
                          char hemisphere, double max_deg, double& value) {
    if (text.size() <= deg_digits) return false;
    for (size_t i = 0; i < deg_digits; ++i)
        if (text[i] < '0' || text[i] > '9') return false;
    int degrees = 0;
    for (size_t i = 0; i < deg_digits; ++i) degrees = degrees * 10 + text[i] - '0';
    double minutes = 0.0;
    if (!parse_decimal(text.substr(deg_digits), minutes) || minutes < 0.0 || minutes >= 60.0)
        return false;
    value = degrees + minutes / 60.0;
    if (value > max_deg || (degrees == static_cast<int>(max_deg) && minutes != 0.0))
        return false;
    if (hemisphere == 'S' || hemisphere == 'W') value = -value;
    return true;
}

}  // namespace detail

/// @brief Parse the exact JSON state object emitted by encode_state_json().
/// @param data Raw datagram bytes; no NUL terminator is required.
/// @param size Number of bytes in @p data.
/// @param out Populated only when the returned status is successful.
/// @return Parse status with a stable, human-readable rejection message.
/// @note Unknown fields are ignored for forward compatibility. All documented
///       scalar fields and the three four-wheel arrays remain mandatory.
inline ParseStatus parse_state_json(const uint8_t* data, size_t size, StateFields& out) {
    if (data == nullptr && size != 0)
        return ParseStatus::failure("json: null input pointer");
    if (size == 0) return ParseStatus::failure("json: empty datagram");
    if (size > 4096) return ParseStatus::failure("json: datagram exceeds 4096 bytes");

    detail::JsonCursor cursor(std::string_view(reinterpret_cast<const char*>(data), size));
    if (!cursor.take('{')) return detail::json_error(cursor, "expected object");
    StateFields candidate{};
    uint32_t seen = 0;
    constexpr uint32_t kAll = (1u << 13u) - 1u;
    cursor.whitespace();
    if (cursor.take('}')) return ParseStatus::failure("json: missing required fields");

    while (true) {
        std::string key;
        if (!cursor.string(key)) return detail::json_error(cursor, "expected field name");
        if (!cursor.take(':')) return detail::json_error(cursor, "expected ':' after field name");

        int index = -1;
        if (key == "id") index = 0;
        else if (key == "t") index = 1;
        else if (key == "x") index = 2;
        else if (key == "y") index = 3;
        else if (key == "yaw") index = 4;
        else if (key == "vx") index = 5;
        else if (key == "vy") index = 6;
        else if (key == "r") index = 7;
        else if (key == "ax") index = 8;
        else if (key == "ay") index = 9;
        else if (key == "Fz") index = 10;
        else if (key == "alpha") index = 11;
        else if (key == "kappa") index = 12;

        if (index < 0) {
            if (!cursor.skip_value()) return detail::json_error(cursor, "invalid value for unknown field '" + key + "'");
        } else {
            const uint32_t bit = 1u << static_cast<uint32_t>(index);
            if ((seen & bit) != 0) return ParseStatus::failure("json: duplicate field '" + key + "'");
            seen |= bit;
            if (index == 0) {
                double id = 0.0;
                bool is_null = false;
                if (!cursor.number(id, is_null) || is_null || id < 0.0 ||
                    id > static_cast<double>(std::numeric_limits<uint32_t>::max()) ||
                    std::floor(id) != id)
                    return detail::json_error(cursor, "field 'id' must be an unsigned integer");
                candidate.vehicle_id = static_cast<uint32_t>(id);
            } else if (index >= 10) {
                double* values = index == 10 ? candidate.Fz :
                                 (index == 11 ? candidate.slip_angle : candidate.slip_ratio);
                if (!cursor.array4(values))
                    return detail::json_error(cursor, "field '" + key + "' must contain exactly four numbers or nulls");
            } else {
                double value = 0.0;
                bool is_null = false;
                if (!cursor.number(value, is_null))
                    return detail::json_error(cursor, "field '" + key + "' must be a number or null");
                double* target = nullptr;
                if (index == 1) target = &candidate.timestamp;
                else if (index == 2) target = &candidate.x;
                else if (index == 3) target = &candidate.y;
                else if (index == 4) target = &candidate.yaw;
                else if (index == 5) target = &candidate.vx;
                else if (index == 6) target = &candidate.vy;
                else if (index == 7) target = &candidate.yaw_rate;
                else if (index == 8) target = &candidate.ax;
                else if (index == 9) target = &candidate.ay;
                *target = value;
            }
        }
        if (cursor.take('}')) break;
        if (!cursor.take(',')) return detail::json_error(cursor, "expected ',' or '}'");
    }
    if (!cursor.at_end()) return detail::json_error(cursor, "trailing characters");
    if (seen != kAll) return ParseStatus::failure("json: missing one or more required state fields");
    out = candidate;
    return ParseStatus::success();
}

/// @brief Parse one NMEA 0183 `$GPGGA` datagram emitted by encode_state_nmea_gga().
/// @param data Raw sentence bytes, including the required `*HH\r\n` terminator.
/// @param size Number of bytes in @p data.
/// @param out Populated only when the returned status is successful.
/// @return Parse status with checksum and field-validation errors separated.
inline ParseStatus parse_nmea_gga(const uint8_t* data, size_t size, GgaFix& out) {
    if (data == nullptr && size != 0)
        return ParseStatus::failure("nmea_gga: null input pointer");
    if (size == 0) return ParseStatus::failure("nmea_gga: empty datagram");
    const std::string sentence(reinterpret_cast<const char*>(data), size);
    if (sentence.size() < 10 || sentence.front() != '$')
        return ParseStatus::failure("nmea_gga: expected '$' sentence start");
    if (sentence.size() < 2 || sentence.substr(sentence.size() - 2) != "\r\n")
        return ParseStatus::failure("nmea_gga: expected CRLF terminator");
    const size_t star = sentence.rfind('*', sentence.size() - 3);
    if (star == std::string::npos || star + 5 != sentence.size())
        return ParseStatus::failure("nmea_gga: expected '*HH' checksum before CRLF");
    uint8_t hi = 0, lo = 0;
    if (!detail::parse_hex(sentence[star + 1], hi) || !detail::parse_hex(sentence[star + 2], lo))
        return ParseStatus::failure("nmea_gga: checksum is not hexadecimal");
    const std::string body = sentence.substr(1, star - 1);
    const uint8_t expected = nmea_checksum(body);
    const uint8_t received = static_cast<uint8_t>((hi << 4u) | lo);
    if (expected != received) {
        char message[96];
        std::snprintf(message, sizeof(message),
                      "nmea_gga: checksum mismatch (expected %02X, got %02X)",
                      expected, received);
        return ParseStatus::failure(message);
    }

    const std::vector<std::string> f = detail::comma_fields(body);
    if (f.size() != 15) return ParseStatus::failure("nmea_gga: expected 15 comma-separated fields");
    if (f[0] != "GPGGA") return ParseStatus::failure("nmea_gga: sentence id must be GPGGA");

    GgaFix candidate;
    if (!detail::parse_utc(f[1], candidate.utc_sod))
        return ParseStatus::failure("nmea_gga: invalid UTC field");
    if (!detail::parse_int(f[6], candidate.fix_quality) ||
        candidate.fix_quality < 0 || candidate.fix_quality > 8)
        return ParseStatus::failure("nmea_gga: fix quality must be in [0,8]");
    if (!detail::parse_int(f[7], candidate.satellites) ||
        candidate.satellites < 0 || candidate.satellites > 99)
        return ParseStatus::failure("nmea_gga: satellites must be in [0,99]");
    if (!detail::parse_decimal(f[8], candidate.hdop) || candidate.hdop < 0.0)
        return ParseStatus::failure("nmea_gga: HDOP must be non-negative");

    if (candidate.has_fix()) {
        if (f[3] != "N" && f[3] != "S")
            return ParseStatus::failure("nmea_gga: latitude hemisphere must be N or S");
        if (f[5] != "E" && f[5] != "W")
            return ParseStatus::failure("nmea_gga: longitude hemisphere must be E or W");
        if (!detail::parse_deg_min(f[2], 2, f[3][0], 90.0, candidate.lat_deg))
            return ParseStatus::failure("nmea_gga: invalid latitude field");
        if (!detail::parse_deg_min(f[4], 3, f[5][0], 180.0, candidate.lon_deg))
            return ParseStatus::failure("nmea_gga: invalid longitude field");
    } else if (!f[2].empty() || !f[3].empty() || !f[4].empty() || !f[5].empty()) {
        return ParseStatus::failure("nmea_gga: no-fix sentence must not contain a position");
    }

    if (f[10] != "M" || f[12] != "M")
        return ParseStatus::failure("nmea_gga: altitude and geoid units must be M");
    if (!f[9].empty() && !detail::parse_decimal(f[9], candidate.alt_m))
        return ParseStatus::failure("nmea_gga: invalid altitude field");
    if (!f[11].empty() && !detail::parse_decimal(f[11], candidate.geoid_sep_m))
        return ParseStatus::failure("nmea_gga: invalid geoid separation field");
    out = candidate;
    return ParseStatus::success();
}

}  // namespace vdsim::cosim
