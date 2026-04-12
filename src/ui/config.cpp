#include "ui/config.hpp"

#include <toml++/toml.h>

namespace auto_battlebot {
void UiConfiguration::parse_fields(ConfigParser &parser) {
    enable = parser.get_optional_bool("enable", enable);
    fullscreen = parser.get_optional_bool("fullscreen", fullscreen);
    width = parser.get_optional_int("width", width);
    height = parser.get_optional_int("height", height);
    rate_avg_window = parser.get_optional_int("rate_avg_window", rate_avg_window);
    rate_fail_threshold = parser.get_optional_double("rate_fail_threshold", rate_fail_threshold);
    rate_fail_duration_sec =
        parser.get_optional_double("rate_fail_duration_sec", rate_fail_duration_sec);
    battery_source = parser.get_optional_string("battery_source", battery_source);

    if (const toml::table *battery_table = parser.get_table("battery")) {
        ConfigParser battery_parser(*battery_table, "ui.battery");
        battery.i2c_bus = battery_parser.get_optional_int("i2c_bus", battery.i2c_bus);
        battery.i2c_address = battery_parser.get_optional_int("i2c_address", battery.i2c_address);
        battery.ocv_table_file_path =
            battery_parser.get_optional_string("ocv_table_file_path", battery.ocv_table_file_path);
        battery.battery_capacity_ah =
            battery_parser.get_optional_double("battery_capacity_ah", battery.battery_capacity_ah);
        battery.rest_current_threshold_a = battery_parser.get_optional_double(
            "rest_current_threshold_a", battery.rest_current_threshold_a);
        battery.rest_stability_delta_v = battery_parser.get_optional_double(
            "rest_stability_delta_v", battery.rest_stability_delta_v);
        battery.rest_window_sec =
            battery_parser.get_optional_double("rest_window_sec", battery.rest_window_sec);
        battery.ocv_correction_gain =
            battery_parser.get_optional_double("ocv_correction_gain", battery.ocv_correction_gain);
        battery.display_slew_pct_per_sec = battery_parser.get_optional_double(
            "display_slew_pct_per_sec", battery.display_slew_pct_per_sec);
        battery.discharge_current_positive = battery_parser.get_optional_bool(
            "discharge_current_positive", battery.discharge_current_positive);
        battery.persist_interval_sec = battery_parser.get_optional_double(
            "persist_interval_sec", battery.persist_interval_sec);
        battery.invalid_read_grace =
            battery_parser.get_optional_int("invalid_read_grace", battery.invalid_read_grace);
        battery.state_file_path =
            battery_parser.get_optional_string("state_file_path", battery.state_file_path);
        battery_parser.validate_no_extra_fields();
    }
}

std::unique_ptr<UiConfiguration> parse_ui_config(ConfigParser &parser) {
    auto config = std::make_unique<UiConfiguration>();
    config->parse_fields(parser);
    parser.validate_no_extra_fields();
    return config;
}

std::unique_ptr<UiConfiguration> load_ui_from_toml(toml::table const &toml_data,
                                                   std::vector<std::string> &parsed_sections) {
    auto section = toml_data["ui"].as_table();
    if (!section) {
        return nullptr;
    }
    ConfigParser parser(*section, "ui");
    auto config = parse_ui_config(parser);
    parsed_sections.push_back("ui");
    return config;
}
}  // namespace auto_battlebot
