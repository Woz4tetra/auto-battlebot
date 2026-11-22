#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"

namespace auto_battlebot {

struct PerformanceMonitorConfiguration {
    bool enable = true;
    double window_s = 5.0;
    double evaluate_period_s = 0.1;
    double publish_period_s = 0.5;
    double link_grace_s = 3.0;
    int min_window_ticks = 25;

    double camera_frames_min_rate_hz = 10.0;
    double camera_frames_max_age_s = 0.5;
    double perception_min_rate_hz = 10.0;
    double perception_max_age_s = 0.5;
    double filter_min_rate_hz = 10.0;
    double filter_max_age_s = 0.5;
    double transmit_min_rate_hz = 10.0;
    double transmit_max_age_s = 0.25;
    double crsf_telemetry_min_rate_hz = 0.1;
    double crsf_telemetry_max_age_s = 10.0;
    double espnow_min_rate_hz = 50.0;
    double espnow_max_age_s = 0.25;
    double espnow_min_ack_percent = 90.0;
    double espnow_max_command_age_ms = 100.0;
    double imu_min_rate_hz = 50.0;
    double imu_max_age_s = 0.25;
    double relay_publish_min_rate_hz = 5.0;
    double relay_publish_max_age_s = 1.0;

    double slew_difference = 0.2;
    double slew_tick_fraction = 0.10;
    double guard_scale_floor = 0.9;
    double guard_tick_fraction = 0.05;
    double calibration_outside_fraction = 0.02;
    double phantom_sigma_m = 1.0;
    double phantom_still_speed_mps = 0.05;
    double phantom_min_duration_s = 1.0;
    double yaw_correlation_min = 0.5;
    double yaw_min_rate_std_radps = 0.5;
    int yaw_min_pairs = 20;
    double tracking_coverage_min = 0.85;

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_BOOL(enable)
        PARSE_FIELD_DOUBLE(window_s)
        PARSE_FIELD_DOUBLE(evaluate_period_s)
        PARSE_FIELD_DOUBLE(publish_period_s)
        PARSE_FIELD_DOUBLE(link_grace_s)
        PARSE_FIELD(min_window_ticks)
        PARSE_FIELD_DOUBLE(camera_frames_min_rate_hz)
        PARSE_FIELD_DOUBLE(camera_frames_max_age_s)
        PARSE_FIELD_DOUBLE(perception_min_rate_hz)
        PARSE_FIELD_DOUBLE(perception_max_age_s)
        PARSE_FIELD_DOUBLE(filter_min_rate_hz)
        PARSE_FIELD_DOUBLE(filter_max_age_s)
        PARSE_FIELD_DOUBLE(transmit_min_rate_hz)
        PARSE_FIELD_DOUBLE(transmit_max_age_s)
        PARSE_FIELD_DOUBLE(crsf_telemetry_min_rate_hz)
        PARSE_FIELD_DOUBLE(crsf_telemetry_max_age_s)
        PARSE_FIELD_DOUBLE(espnow_min_rate_hz)
        PARSE_FIELD_DOUBLE(espnow_max_age_s)
        PARSE_FIELD_DOUBLE(espnow_min_ack_percent)
        PARSE_FIELD_DOUBLE(espnow_max_command_age_ms)
        PARSE_FIELD_DOUBLE(imu_min_rate_hz)
        PARSE_FIELD_DOUBLE(imu_max_age_s)
        PARSE_FIELD_DOUBLE(relay_publish_min_rate_hz)
        PARSE_FIELD_DOUBLE(relay_publish_max_age_s)
        PARSE_FIELD_DOUBLE(slew_difference)
        PARSE_FIELD_DOUBLE(slew_tick_fraction)
        PARSE_FIELD_DOUBLE(guard_scale_floor)
        PARSE_FIELD_DOUBLE(guard_tick_fraction)
        PARSE_FIELD_DOUBLE(calibration_outside_fraction)
        PARSE_FIELD_DOUBLE(phantom_sigma_m)
        PARSE_FIELD_DOUBLE(phantom_still_speed_mps)
        PARSE_FIELD_DOUBLE(phantom_min_duration_s)
        PARSE_FIELD_DOUBLE(yaw_correlation_min)
        PARSE_FIELD_DOUBLE(yaw_min_rate_std_radps)
        PARSE_FIELD(yaw_min_pairs)
        PARSE_FIELD_DOUBLE(tracking_coverage_min)
    )
    // clang-format on
};

PerformanceMonitorConfiguration load_performance_monitor_from_toml(
    const toml::table &toml_data, std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
