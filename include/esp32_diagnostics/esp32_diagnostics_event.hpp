#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace auto_battlebot {

/**
 * One line of the Mr Stabs Mk2 firmware's diagnostics stream, as `DiagnosticsServer::update`
 * (firmware/mr_stabs_mk2/src/diagnostics_server.cpp) prints it, plus the host time it arrived.
 */
struct Esp32DiagnosticsEvent {
    /** Host wall clock (the MCAP log time base) when the line arrived, in nanoseconds. */
    uint64_t host_receive_ns = 0;
    /** Robot clock, millis() on the ESP32. */
    uint64_t timestamp_ms = 0;
    bool radio_connected = false;
    bool armed = false;
    double a_percent = 0.0;
    double b_percent = 0.0;
    bool button_state = false;
    int flip_switch = 0;
    /** Per-motor commands after the heading PID and the mixer, percent. */
    double left_cmd = 0.0;
    double right_cmd = 0.0;
    double accel_x = 0.0;
    double accel_y = 0.0;
    double accel_z = 0.0;
    bool is_upside_down = false;
    int64_t loop_us = 0;
    int wifi_clients = 0;
    /** BNO055 Euler angles, degrees. */
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double pid_setpoint = 0.0;
    double pid_output = 0.0;
    /** Pack voltage from the INA219, volts. Empty on firmware that predates the field and when
     *  the firmware prints `nan` because the INA219 is missing or a read failed. */
    std::optional<double> vbat;
};

/** Field count before the firmware added `vbat`. */
constexpr int kEsp32DiagnosticsFieldsWithoutVbat = 20;
/** Field count with `vbat` appended. */
constexpr int kEsp32DiagnosticsFieldsWithVbat = 21;

/**
 * Parse one CSV event line. Accepts 20 fields (no `vbat`) or 21. Returns empty for any other
 * field count or for a field that does not parse as its type.
 */
std::optional<Esp32DiagnosticsEvent> parse_esp32_diagnostics_csv(std::string_view line,
                                                                 uint64_t host_receive_ns);

/** The `/robot/esp32_diagnostics` JSON payload for one event. Non-finite numbers become null. */
std::string to_esp32_diagnostics_json(const Esp32DiagnosticsEvent &event);

}  // namespace auto_battlebot
