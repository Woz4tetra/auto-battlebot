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

    /** UI window/config from [ui] section. Used when runner.ui_enabled is true. */
    struct UIConfiguration
    {
        int width = 1280;
        int height = 800;

        UIConfiguration() = default;
    };
} // namespace auto_battlebot
