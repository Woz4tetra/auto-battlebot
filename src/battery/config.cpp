#include "battery/config.hpp"

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"

namespace auto_battlebot {

BatteryConfiguration load_battery_from_toml(const toml::table &toml_data,
                                            std::vector<std::string> &parsed_sections) {
    BatteryConfiguration config;
    auto section = toml_data["battery"].as_table();
    if (!section) {
        return config;
    }

    ConfigParser parser(*section, "battery");
    BatterySource source = config.source;
    PARSE_ENUM(source, BatterySource)
    config.source = source;

    BatteryOptions &o = config.options;
    o.i2c_bus = static_cast<int>(parser.get_optional_int("i2c_bus", o.i2c_bus));
    o.i2c_address = static_cast<int>(parser.get_optional_int("i2c_address", o.i2c_address));
    o.ocv_table_file_path =
        parser.get_optional_string("ocv_table_file_path", o.ocv_table_file_path);
    o.battery_capacity_ah =
        parser.get_optional_double("battery_capacity_ah", o.battery_capacity_ah);
    o.rest_current_threshold_a =
        parser.get_optional_double("rest_current_threshold_a", o.rest_current_threshold_a);
    o.rest_stability_delta_v =
        parser.get_optional_double("rest_stability_delta_v", o.rest_stability_delta_v);
    o.rest_window_sec = parser.get_optional_double("rest_window_sec", o.rest_window_sec);
    o.ocv_correction_gain =
        parser.get_optional_double("ocv_correction_gain", o.ocv_correction_gain);
    o.display_slew_pct_per_sec =
        parser.get_optional_double("display_slew_pct_per_sec", o.display_slew_pct_per_sec);
    o.discharge_current_positive =
        parser.get_optional_bool("discharge_current_positive", o.discharge_current_positive);
    o.persist_interval_sec =
        parser.get_optional_double("persist_interval_sec", o.persist_interval_sec);
    o.invalid_read_grace =
        static_cast<int>(parser.get_optional_int("invalid_read_grace", o.invalid_read_grace));
    o.state_file_path = parser.get_optional_string("state_file_path", o.state_file_path);

    parser.validate_no_extra_fields();
    parsed_sections.push_back("battery");
    return config;
}

}  // namespace auto_battlebot
