#include "time/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "time/manual_clock.hpp"
#include "time/system_clock.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(ClockConfiguration, SystemClockConfiguration, "SystemClock")
REGISTER_CONFIG(ClockConfiguration, ManualClockConfiguration, "ManualClock")

std::unique_ptr<ClockConfiguration> parse_clock_config(ConfigParser &parser) {
    return ConfigFactory<ClockConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<ClockConfiguration> load_clock_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["clock"].as_table();
    if (!section) {
        // Optional section: default to the wall-clock SystemClock (hardware/playback behavior
        // unchanged).
        return std::make_unique<SystemClockConfiguration>();
    }
    ConfigParser parser(*section, "clock");
    auto config = parse_clock_config(parser);
    parsed_sections.push_back("clock");
    return config;
}

std::shared_ptr<ClockInterface> make_clock(const ClockConfiguration &config) {
    spdlog::info("Selected {} for Clock", config.type);
    if (config.type == "SystemClock") {
        return std::make_shared<SystemClock>();
    }
    if (config.type == "ManualClock") {
        return std::make_shared<ManualClock>();
    }
    throw std::invalid_argument("Failed to load Clock of type " + config.type);
}
}  // namespace auto_battlebot
