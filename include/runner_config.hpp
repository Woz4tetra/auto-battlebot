#pragma once

#include <cstdint>

namespace auto_battlebot
{
    struct RunnerConfiguration
    {
        double max_loop_rate; // Hz
        bool ui_enabled;

        RunnerConfiguration() : max_loop_rate(300.0), ui_enabled(false) {}
    };
} // namespace auto_battlebot
