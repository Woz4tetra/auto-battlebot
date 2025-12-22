#pragma once

#include "data_structures.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "config_factory.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    struct RobotFilterConfiguration
    {
        std::string type;
        virtual ~RobotFilterConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopRobotFilterConfiguration : public RobotFilterConfiguration
    {
        NoopRobotFilterConfiguration()
        {
            type = "NoopRobotFilter";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config);
    std::unique_ptr<RobotFilterConfiguration> parse_robot_filter_config(ConfigParser &parser);
} // namespace auto_battlebot
