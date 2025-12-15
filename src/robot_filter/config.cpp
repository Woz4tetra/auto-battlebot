#include "robot_filter/config.hpp"
#include "robot_filter/noop_robot_filter.hpp"

namespace auto_battlebot
{
    std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config)
    {
        if (config.type == "NoopRobotFilter")
        {
            return std::make_shared<NoopRobotFilter>();
        }
        return nullptr;
    }
} // namespace auto_battlebot
