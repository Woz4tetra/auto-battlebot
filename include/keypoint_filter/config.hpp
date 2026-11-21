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

struct KeypointFilterConfiguration {
    KeypointHeightGateConfiguration height;
};

KeypointFilterConfiguration load_keypoint_filter_from_toml(
    const toml::table &toml_data, std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
