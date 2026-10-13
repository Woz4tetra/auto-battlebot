#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

#include "battery/battery_options.hpp"
#include "enums/battery_source.hpp"

namespace auto_battlebot {

/** [battery]: `source` plus the estimator settings in BatteryOptions, all in one table. */
struct BatteryConfiguration {
    BatterySource source = BatterySource::NONE;
    BatteryOptions options;
};

BatteryConfiguration load_battery_from_toml(const toml::table &toml_data,
                                            std::vector<std::string> &parsed_sections);

}  // namespace auto_battlebot
