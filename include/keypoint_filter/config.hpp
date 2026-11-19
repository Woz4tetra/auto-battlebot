#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

namespace auto_battlebot {

struct KeypointHeightGateConfiguration {
    bool reject_enable = true;
    double min_elevation_meters = 0.035;
    double max_height_meters = 0.40;
    double top_percentile = 0.90;
    double floor_percentile = 0.25;
    double ring_inner_scale = 1.4;
    double ring_outer_scale = 2.4;
    int sample_radius_px = 5;
    int min_valid_samples = 8;
    int max_circle_samples = 1024;
};

struct StaticDetectionGateConfiguration {
    bool enable = false;
    double match_radius_meters = 0.20;
    double static_radius_meters = 0.080;
    double min_dwell_seconds = 4.0;
    int min_observations = 30;
    int responsiveness_window = 60;
    double forget_seconds = 2.0;
};

struct KeypointFilterConfiguration {
    KeypointHeightGateConfiguration height;
    StaticDetectionGateConfiguration static_gate;
};

KeypointFilterConfiguration load_keypoint_filter_from_toml(
    const toml::table &toml_data, std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
