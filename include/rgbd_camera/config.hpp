#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    struct RgbdCameraConfiguration
    {
        std::string type;
    };

    struct NoopRgbdCameraConfiguration : public RgbdCameraConfiguration
    {
        NoopRgbdCameraConfiguration()
        {
            type = "NoopRgbdCamera";
        }
    };
} // namespace auto_battlebot
