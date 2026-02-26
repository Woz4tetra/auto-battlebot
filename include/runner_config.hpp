#pragma once

#include <cstdint>

namespace auto_battlebot
{
    struct RunnerConfiguration
    {
        double max_loop_rate; // Hz

        RunnerConfiguration() : max_loop_rate(300.0) {}
    };

    struct UIConfiguration
    {
        bool enable = false;
        bool fullscreen = true;
        int width = 1280;
        int height = 800;

        UIConfiguration() = default;
    };
} // namespace auto_battlebot
