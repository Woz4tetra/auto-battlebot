#include "keypoint_filter/config.hpp"

#include <string>
#include <utility>

#include "config/config_parser.hpp"

namespace auto_battlebot {
namespace {

void parse_height(const toml::table &section, KeypointHeightGateConfiguration &config) {
    ConfigParser parser(section, "keypoint_filter.height");
    config.reject_enable = parser.get_optional_bool("reject_enable", config.reject_enable);
    config.min_elevation_meters =
        parser.get_optional_double("min_elevation_meters", config.min_elevation_meters);
    config.max_height_meters =
        parser.get_optional_double("max_height_meters", config.max_height_meters);
    config.top_percentile = parser.get_optional_double("top_percentile", config.top_percentile);
    config.floor_percentile =
        parser.get_optional_double("floor_percentile", config.floor_percentile);
    config.ring_inner_scale =
        parser.get_optional_double("ring_inner_scale", config.ring_inner_scale);
    config.ring_outer_scale =
        parser.get_optional_double("ring_outer_scale", config.ring_outer_scale);
    config.sample_radius_px = static_cast<int>(
        parser.get_optional_int("sample_radius_px", static_cast<int64_t>(config.sample_radius_px)));
    config.min_valid_samples = static_cast<int>(parser.get_optional_int(
        "min_valid_samples", static_cast<int64_t>(config.min_valid_samples)));
    config.max_circle_samples = static_cast<int>(parser.get_optional_int(
        "max_circle_samples", static_cast<int64_t>(config.max_circle_samples)));
    parser.validate_no_extra_fields();

    if (config.min_elevation_meters >= config.max_height_meters) {
        throw ConfigValidationError(
            "keypoint_filter.height.min_elevation_meters must be < max_height_meters");
    }
    for (const auto &[name, value] : {std::pair{"top_percentile", config.top_percentile},
                                      std::pair{"floor_percentile", config.floor_percentile}}) {
        if (value < 0.0 || value > 1.0) {
            throw ConfigValidationError(std::string("keypoint_filter.height.") + name +
                                        " must be in [0, 1]");
        }
    }
    if (config.ring_inner_scale < 1.0) {
        throw ConfigValidationError("keypoint_filter.height.ring_inner_scale must be >= 1");
    }
    if (config.ring_outer_scale <= config.ring_inner_scale) {
        throw ConfigValidationError(
            "keypoint_filter.height.ring_outer_scale must be > ring_inner_scale");
    }
    if (config.sample_radius_px < 0) {
        throw ConfigValidationError("keypoint_filter.height.sample_radius_px must be >= 0");
    }
    if (config.min_valid_samples < 1) {
        throw ConfigValidationError("keypoint_filter.height.min_valid_samples must be >= 1");
    }
    if (config.max_circle_samples < 1) {
        throw ConfigValidationError("keypoint_filter.height.max_circle_samples must be >= 1");
    }
}

void parse_static(const toml::table &section, StaticDetectionGateConfiguration &config) {
    ConfigParser parser(section, "keypoint_filter.static");
    config.enable = parser.get_optional_bool("enable", config.enable);
    config.match_radius_meters =
        parser.get_optional_double("match_radius_meters", config.match_radius_meters);
    config.static_radius_meters =
        parser.get_optional_double("static_radius_meters", config.static_radius_meters);
    config.min_dwell_seconds =
        parser.get_optional_double("min_dwell_seconds", config.min_dwell_seconds);
    config.min_observations = static_cast<int>(
        parser.get_optional_int("min_observations", static_cast<int64_t>(config.min_observations)));
    config.responsiveness_window = static_cast<int>(parser.get_optional_int(
        "responsiveness_window", static_cast<int64_t>(config.responsiveness_window)));
    config.forget_seconds = parser.get_optional_double("forget_seconds", config.forget_seconds);
    parser.validate_no_extra_fields();

    if (config.match_radius_meters <= 0.0) {
        throw ConfigValidationError("keypoint_filter.static.match_radius_meters must be > 0");
    }
    if (config.static_radius_meters <= 0.0) {
        throw ConfigValidationError("keypoint_filter.static.static_radius_meters must be > 0");
    }
    if (config.static_radius_meters > config.match_radius_meters) {
        throw ConfigValidationError(
            "keypoint_filter.static.static_radius_meters must be <= match_radius_meters");
    }
    if (config.min_dwell_seconds <= 0.0) {
        throw ConfigValidationError("keypoint_filter.static.min_dwell_seconds must be > 0");
    }
    if (config.min_observations < 1) {
        throw ConfigValidationError("keypoint_filter.static.min_observations must be >= 1");
    }
    if (config.responsiveness_window < 1) {
        throw ConfigValidationError("keypoint_filter.static.responsiveness_window must be >= 1");
    }
    if (config.forget_seconds <= 0.0) {
        throw ConfigValidationError("keypoint_filter.static.forget_seconds must be > 0");
    }
}

}  // namespace

KeypointFilterConfiguration load_keypoint_filter_from_toml(
    const toml::table &toml_data, std::vector<std::string> &parsed_sections) {
    KeypointFilterConfiguration config;

    const auto *section = toml_data["keypoint_filter"].as_table();
    if (!section) return config;

    ConfigParser parser(*section, "keypoint_filter");
    if (const toml::table *height = parser.get_table("height")) {
        parse_height(*height, config.height);
    }
    if (const toml::table *static_section = parser.get_table("static")) {
        parse_static(*static_section, config.static_gate);
    }
    parser.validate_no_extra_fields();

    parsed_sections.push_back("keypoint_filter");
    return config;
}

}  // namespace auto_battlebot
