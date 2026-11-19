#include "health/config.hpp"

#include "config/config_parser.hpp"

namespace auto_battlebot {

HealthConfiguration load_health_from_toml(const toml::table& toml_data,
                                          std::vector<std::string>& parsed_sections) {
    HealthConfiguration config;
    auto section = toml_data["health"].as_table();
    if (!section) {
        return config;
    }

    ConfigParser parser(*section, "health");
    config.enable = parser.get_optional_bool("enable", config.enable);
    config.tegrastats_enable =
        parser.get_optional_bool("tegrastats_enable", config.tegrastats_enable);
    config.x86_tools_enable = parser.get_optional_bool("x86_tools_enable", config.x86_tools_enable);
    config.sample_period_ms = static_cast<int>(
        parser.get_optional_int("sample_period_ms", static_cast<int64_t>(config.sample_period_ms)));
    if (config.sample_period_ms <= 0) {
        throw ConfigValidationError("health.sample_period_ms must be > 0");
    }

    config.disk_used_warn_percent =
        parser.get_optional_double("disk_used_warn_percent", config.disk_used_warn_percent);
    if (config.disk_used_warn_percent <= 0.0 || config.disk_used_warn_percent > 100.0) {
        throw ConfigValidationError("health.disk_used_warn_percent must be in (0, 100]");
    }

    parser.validate_no_extra_fields();
    parsed_sections.push_back("health");
    return config;
}

}  // namespace auto_battlebot
