#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace auto_battlebot {

enum class MotorGuardState : uint8_t { OFF, NOMINAL, HOT, LIMITING, TRIPPED };
enum class MotorGuardSensors : uint8_t { FUSED, SHUNT_ONLY, ERPM_ONLY, NONE };

struct Esp32MotorGuard {
    double motor_rise_left_c = 0.0;
    double motor_rise_right_c = 0.0;
    double motor_speed_left_rad_s = 0.0;
    double motor_speed_right_rad_s = 0.0;
    double motor_current_left_a = 0.0;
    double motor_current_right_a = 0.0;
    double guard_scale = 1.0;
    double guard_gain = 1.0;
    MotorGuardState guard_state = MotorGuardState::OFF;
    MotorGuardSensors guard_sensors = MotorGuardSensors::FUSED;
    bool left_stalled = false;
    bool right_stalled = false;
    uint32_t erpm_crc_errors_since_boot = 0;
};

struct Esp32DiagnosticsEvent {
    uint64_t host_receive_wall_ns = 0;
    uint64_t robot_millis = 0;
    bool radio_connected = false;
    bool armed = false;
    double a_percent = 0.0;
    double b_percent = 0.0;
    bool button_state = false;
    int flip_switch = 0;
    double left_cmd_post_mixer_pct = 0.0;
    double right_cmd_post_mixer_pct = 0.0;
    double accel_x = 0.0;
    double accel_y = 0.0;
    double accel_z = 0.0;
    bool is_upside_down = false;
    int64_t loop_us = 0;
    int wifi_clients = 0;
    double orientation_x_deg = 0.0;
    double orientation_y_deg = 0.0;
    double orientation_z_deg = 0.0;
    double pid_setpoint = 0.0;
    double pid_output = 0.0;
    std::optional<double> vbat;
    std::optional<double> ibat;
    std::optional<double> yaw_rate_cw_deg_s;
    std::optional<double> yaw_rate_cmd_cw_deg_s;
    std::optional<Esp32MotorGuard> guard;
    std::optional<uint32_t> seq;
    std::optional<bool> backfill;
};

constexpr int kEsp32DiagnosticsFieldsWithoutVbat = 20;
constexpr int kEsp32DiagnosticsFieldsWithVbat = 21;
constexpr int kEsp32DiagnosticsFieldsWithIbat = 22;
constexpr int kEsp32DiagnosticsFieldsWithYawRate = 24;
constexpr int kEsp32DiagnosticsFieldsWithGuard = 37;

std::optional<Esp32DiagnosticsEvent> parse_esp32_diagnostics_csv(std::string_view line,
                                                                 uint64_t host_receive_wall_ns);

std::string to_esp32_diagnostics_json(const Esp32DiagnosticsEvent &event);

}  // namespace auto_battlebot
