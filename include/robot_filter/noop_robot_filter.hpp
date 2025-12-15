#pragma once

#include "robot_filter/robot_filter_interface.hpp"

namespace auto_battlebot
{
    class NoopRobotFilter : public RobotFilterInterface
    {
    public:
        bool initialize([[maybe_unused]] const std::vector<RobotConfig> &robots) override { return true; }

        RobotDescriptionsStamped update([[maybe_unused]] KeypointsStamped keypoints, [[maybe_unused]] FieldDescription field) override
        {
            return RobotDescriptionsStamped{};
        }
    };

} // namespace auto_battlebot
