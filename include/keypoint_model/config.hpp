#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    struct KeypointModelConfiguration
    {
        std::string type;
    };

    struct NoopKeypointModelConfiguration : public KeypointModelConfiguration
    {
        NoopKeypointModelConfiguration()
        {
            type = "NoopKeypointModel";
        }
    };
} // namespace auto_battlebot
