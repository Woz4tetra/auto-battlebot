#include "robot_filter/config.hpp"
#include "robot_filter/noop_robot_filter.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(RobotFilterConfiguration, NoopRobotFilterConfiguration, "NoopRobotFilter")

    std::unique_ptr<RobotFilterConfiguration> parse_robot_filter_config(ConfigParser &parser)
    {
        return ConfigFactory<RobotFilterConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config)
    {
        if (config.type == "NoopRobotFilter")
        {
            return std::make_shared<NoopRobotFilter>();
        }
        throw std::invalid_argument("Failed to load RobotFilter of type " + config.type);
    }
} // namespace auto_battlebot
