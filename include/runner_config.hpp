#pragma once

#include <cstdint>

namespace auto_battlebot
{
    struct RunnerConfiguration
    {
        double max_loop_rate; // Hz

        RunnerConfiguration() : max_loop_rate(1000.0) {}
    };
} // namespace auto_battlebot
