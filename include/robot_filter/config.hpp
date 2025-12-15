#pragma once

#include "data_structures.hpp"
#include "robot_filter/robot_filter_interface.hpp"

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

    std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config);
} // namespace auto_battlebot
