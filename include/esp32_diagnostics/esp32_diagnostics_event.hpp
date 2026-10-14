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
    /** Pack voltage from the INA228, volts. Empty on firmware that predates the field and when
     *  the firmware prints `nan` because the INA228 is missing or a read failed. */
    std::optional<double> vbat;
    /** Pack current from the INA228, amps, positive while discharging. Empty under the same
     *  conditions as `vbat`. */
    std::optional<double> ibat;
    /** Gyro yaw rate, deg/s clockwise. Empty on firmware that predates the field. */
    std::optional<double> yaw_rate;
    /** Rate the yaw loop commands, deg/s clockwise. Empty when the loop is not running (the
     *  firmware prints `nan`) and on firmware that predates the field. */
    std::optional<double> yaw_rate_cmd;
    /** Firmware row counter, +1 per row sent. A gap means rows were dropped between the robot and
     *  the host. Empty on firmware that predates the field. */
    std::optional<uint32_t> seq;
    /** Set on rows read from the firmware's telemetry ring (`GET /telemetry`): true when the row
     *  arrived later than it would have live, because it waited in the ring for the poll or for
     *  the link to come back. Its `host_receive_ns` then overstates the delivery delay, so clock
     *  fits skip it. Empty on rows from the event stream. */
    std::optional<bool> backfill;
};

/** Field count before the firmware added `vbat`. */
constexpr int kEsp32DiagnosticsFieldsWithoutVbat = 20;
/** Field count with `vbat` appended. */
constexpr int kEsp32DiagnosticsFieldsWithVbat = 21;
/** Field count with `ibat` appended after `vbat`. */
constexpr int kEsp32DiagnosticsFieldsWithIbat = 22;
/** Field count with `yaw_rate` and `yaw_rate_cmd` appended after `ibat`. */
constexpr int kEsp32DiagnosticsFieldsWithYawRate = 24;
/** Field count with `seq` appended after `yaw_rate_cmd`. */
constexpr int kEsp32DiagnosticsFieldsWithSeq = 25;

/**
 * Parse one CSV event line. Accepts 20 fields (no `vbat`), 21 (no `ibat`), 22 (no yaw rates), 24
 * (no `seq`), or 25. Returns empty for any other field count or for a field that does not parse as
 * its type.
 */
std::optional<Esp32DiagnosticsEvent> parse_esp32_diagnostics_csv(std::string_view line,
                                                                 uint64_t host_receive_ns);

/** The `/robot/esp32_diagnostics` JSON payload for one event. Non-finite numbers become null. */
std::string to_esp32_diagnostics_json(const Esp32DiagnosticsEvent &event);

}  // namespace auto_battlebot
