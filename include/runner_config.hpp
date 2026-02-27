#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <toml++/toml.h>

namespace auto_battlebot
{
    struct RunnerConfiguration
    {
        double max_loop_rate; // Hz

        RunnerConfiguration() : max_loop_rate(300.0) {}
    };

    void load_runner_from_toml(
        toml::table const &toml_data,
        std::vector<std::string> &parsed_sections,
        RunnerConfiguration &out);
} // namespace auto_battlebot
