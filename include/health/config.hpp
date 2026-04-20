#pragma once

#include <toml++/toml.h>

#include <string>
#include <vector>

namespace auto_battlebot {

struct HealthConfiguration {
    bool enable = false;
    bool tegrastats_enable = false;
    bool x86_tools_enable = false;
    int sample_period_ms = 5000;
};

HealthConfiguration load_health_from_toml(const toml::table& toml_data,
                                          std::vector<std::string>& parsed_sections);

}  // namespace auto_battlebot
