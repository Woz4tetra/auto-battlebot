#pragma once

#include <cstdint>

namespace auto_battlebot
{
    struct RunnerConfiguration
    {
        double loop_rate; // Hz

        RunnerConfiguration() : loop_rate(300.0) {}
    };
} // namespace auto_battlebot
