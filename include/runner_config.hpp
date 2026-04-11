#pragma once

#include <toml++/toml.h>

#include <cstdint>
#include <string>
#include <vector>

namespace auto_battlebot {
struct RunnerConfiguration {
    double max_loop_rate;  // Hz
    int default_opponent_count;
    bool autonomy_enabled_by_default;

    RunnerConfiguration()
        : max_loop_rate(300.0), default_opponent_count(1), autonomy_enabled_by_default(true) {}
};

void load_runner_from_toml(toml::table const &toml_data, std::vector<std::string> &parsed_sections,
                           RunnerConfiguration &out);
}  // namespace auto_battlebot
