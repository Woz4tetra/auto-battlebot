#include "esp32_diagnostics/esp32_diagnostics_event.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <vector>

namespace auto_battlebot {
namespace {

std::string_view trim(std::string_view text) {
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) text.remove_prefix(1);
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t' || text.back() == '\r' ||
                             text.back() == '\n')) {
        text.remove_suffix(1);
    }
    return text;
}

template <typename T>
bool parse_number(std::string_view text, T &out) {
    text = trim(text);
    if (text.empty()) return false;
    const char *end = text.data() + text.size();
    const auto result = std::from_chars(text.data(), end, out);
    return result.ec == std::errc() && result.ptr == end;
}

bool parse_bool(std::string_view text, bool &out) {
    int64_t value = 0;
    if (!parse_number(text, value)) return false;
    out = value != 0;
    return true;
}

template <typename T>
bool parse_int(std::string_view text, T &out) {
    int64_t value = 0;
    if (!parse_number(text, value)) return false;
    out = static_cast<T>(value);
    return true;
}

void append_double(std::string &json, const char *key, double value) {
    json += '"';
    json += key;
    json += "\":";
    if (!std::isfinite(value)) {
        json += "null";
        return;
    }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    json += buffer;
}

void append_int(std::string &json, const char *key, int64_t value) {
    json += '"';
    json += key;
    json += "\":";
    json += std::to_string(value);
}

void append_uint(std::string &json, const char *key, uint64_t value) {
    json += '"';
    json += key;
    json += "\":";
    json += std::to_string(value);
}

void append_bool(std::string &json, const char *key, bool value) {
    json += '"';
    json += key;
    json += "\":";
    json += value ? "true" : "false";
}

}  // namespace

std::optional<Esp32DiagnosticsEvent> parse_esp32_diagnostics_csv(std::string_view line,
                                                                 uint64_t host_receive_ns) {
    line = trim(line);
    std::vector<std::string_view> fields;
    fields.reserve(kEsp32DiagnosticsFieldsWithIbat);
    size_t start = 0;
    while (true) {
        const size_t comma = line.find(',', start);
        if (comma == std::string_view::npos) {
            fields.push_back(line.substr(start));
            break;
        }
        fields.push_back(line.substr(start, comma - start));
        start = comma + 1;
    }
    const int count = static_cast<int>(fields.size());
    if (count != kEsp32DiagnosticsFieldsWithoutVbat && count != kEsp32DiagnosticsFieldsWithVbat &&
        count != kEsp32DiagnosticsFieldsWithIbat) {
        return std::nullopt;
    }

    Esp32DiagnosticsEvent event;
    event.host_receive_ns = host_receive_ns;
    const bool ok =
        parse_number(fields[0], event.timestamp_ms) &&
        parse_bool(fields[1], event.radio_connected) && parse_bool(fields[2], event.armed) &&
        parse_number(fields[3], event.a_percent) && parse_number(fields[4], event.b_percent) &&
        parse_bool(fields[5], event.button_state) && parse_int(fields[6], event.flip_switch) &&
        parse_number(fields[7], event.left_cmd) && parse_number(fields[8], event.right_cmd) &&
        parse_number(fields[9], event.accel_x) && parse_number(fields[10], event.accel_y) &&
        parse_number(fields[11], event.accel_z) && parse_bool(fields[12], event.is_upside_down) &&
        parse_number(fields[13], event.loop_us) && parse_int(fields[14], event.wifi_clients) &&
        parse_number(fields[15], event.orientation_x) &&
        parse_number(fields[16], event.orientation_y) &&
        parse_number(fields[17], event.orientation_z) &&
        parse_number(fields[18], event.pid_setpoint) && parse_number(fields[19], event.pid_output);
    if (!ok) return std::nullopt;

    // The firmware prints `nan` when the INA228 is absent or a read failed.
    if (count >= kEsp32DiagnosticsFieldsWithVbat) {
        double vbat = 0.0;
        if (!parse_number(fields[20], vbat)) return std::nullopt;
        if (std::isfinite(vbat)) event.vbat = vbat;
    }
    if (count >= kEsp32DiagnosticsFieldsWithIbat) {
        double ibat = 0.0;
        if (!parse_number(fields[21], ibat)) return std::nullopt;
        if (std::isfinite(ibat)) event.ibat = ibat;
    }
    return event;
}

std::string to_esp32_diagnostics_json(const Esp32DiagnosticsEvent &event) {
    std::string json;
    json.reserve(640);
    json += '{';
    append_uint(json, "host_receive_ns", event.host_receive_ns);
    json += ',';
    append_uint(json, "timestamp_ms", event.timestamp_ms);
    json += ',';
    append_bool(json, "radio_connected", event.radio_connected);
    json += ',';
    append_bool(json, "armed", event.armed);
    json += ',';
    append_double(json, "a_percent", event.a_percent);
    json += ',';
    append_double(json, "b_percent", event.b_percent);
    json += ',';
    append_bool(json, "button_state", event.button_state);
    json += ',';
    append_int(json, "flip_switch", event.flip_switch);
    json += ',';
    append_double(json, "left_cmd", event.left_cmd);
    json += ',';
    append_double(json, "right_cmd", event.right_cmd);
    json += ',';
    append_double(json, "accel_x", event.accel_x);
    json += ',';
    append_double(json, "accel_y", event.accel_y);
    json += ',';
    append_double(json, "accel_z", event.accel_z);
    json += ',';
    append_bool(json, "is_upside_down", event.is_upside_down);
    json += ',';
    append_int(json, "loop_us", event.loop_us);
    json += ',';
    append_int(json, "wifi_clients", event.wifi_clients);
    json += ',';
    append_double(json, "orientation_x", event.orientation_x);
    json += ',';
    append_double(json, "orientation_y", event.orientation_y);
    json += ',';
    append_double(json, "orientation_z", event.orientation_z);
    json += ',';
    append_double(json, "pid_setpoint", event.pid_setpoint);
    json += ',';
    append_double(json, "pid_output", event.pid_output);
    json += ',';
    if (event.vbat) {
        append_double(json, "vbat", *event.vbat);
    } else {
        json += "\"vbat\":null";
    }
    json += ',';
    if (event.ibat) {
        append_double(json, "ibat", *event.ibat);
    } else {
        json += "\"ibat\":null";
    }
    json += '}';
    return json;
}

}  // namespace auto_battlebot
