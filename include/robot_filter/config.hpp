#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    struct RobotFilterConfiguration
    {
        std::string type;
    };

    struct NoopRobotFilterConfiguration : public RobotFilterConfiguration
    {
        NoopRobotFilterConfiguration()
        {
            type = "NoopRobotFilter";
        }
    };
} // namespace auto_battlebot
