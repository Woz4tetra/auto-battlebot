#include "robot_filter/config.hpp"
#include "robot_filter/noop_robot_filter.hpp"
#include "robot_filter/robot_kalman_filter.hpp"
#include "config/config_parser.hpp"
#include "config/config_cast.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(RobotFilterConfiguration, NoopRobotFilterConfiguration, "NoopRobotFilter")
    REGISTER_CONFIG(RobotFilterConfiguration, RobotKalmanFilterConfiguration, "RobotKalmanFilter")

    std::unique_ptr<RobotFilterConfiguration> parse_robot_filter_config(ConfigParser &parser)
    {
        return ConfigFactory<RobotFilterConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for RobotFilter" << std::endl;
        if (config.type == "NoopRobotFilter")
        {
            return std::make_shared<NoopRobotFilter>();
        }
        else if (config.type == "RobotKalmanFilter")
        {
            return std::make_shared<RobotKalmanFilter>(
                config_cast<RobotKalmanFilterConfiguration>(config));
        }
        throw std::invalid_argument("Failed to load RobotFilter of type " + config.type);
    }
} // namespace auto_battlebot
