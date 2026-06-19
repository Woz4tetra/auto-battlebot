#pragma once

#include <memory>
#include <string>
#include <vector>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct ClockConfiguration {
    std::string type;
    virtual ~ClockConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct SystemClockConfiguration : public ClockConfiguration {
    SystemClockConfiguration() { type = "SystemClock"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct ManualClockConfiguration : public ClockConfiguration {
    ManualClockConfiguration() { type = "ManualClock"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

std::shared_ptr<ClockInterface> make_clock(const ClockConfiguration &config);
std::unique_ptr<ClockConfiguration> parse_clock_config(ConfigParser &parser);
std::unique_ptr<ClockConfiguration> load_clock_from_toml(toml::table const &toml_data,
                                                         std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
