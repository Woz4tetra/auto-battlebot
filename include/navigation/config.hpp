#pragma once

#include "data_structures.hpp"
#include "navigation/navigation_interface.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"

namespace auto_battlebot
{
    struct NavigationConfiguration
    {
        std::string type;
        virtual ~NavigationConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopNavigationConfiguration : public NavigationConfiguration
    {
        NoopNavigationConfiguration()
        {
            type = "NoopNavigation";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    struct PursuitNavigationConfiguration : public NavigationConfiguration
    {
        /** Maximum linear velocity in m/s */
        double max_linear_velocity = 1.0;

        /** Maximum angular velocity in rad/s */
        double max_angular_velocity = 3.0;

        /** Distance at which to start slowing down in meters */
        double slowdown_distance = 0.5;

        /** Distance at which to stop in meters */
        double stop_distance = 0.1;

        /** Proportional gain for angular control */
        double angular_kp = 2.0;

        /** Angle threshold below which robot drives forward (rad) */
        double angle_threshold = 0.3;

        /** Lookahead time for target position prediction in seconds */
        double lookahead_time = 0.1;

        /** Minimum distance to maintain from field boundaries in meters */
        double boundary_margin = 0.1;

        PursuitNavigationConfiguration()
        {
            type = "PursuitNavigation";
        }

        PARSE_CONFIG_FIELDS(
            PARSE_FIELD_DOUBLE(max_linear_velocity)
            PARSE_FIELD_DOUBLE(max_angular_velocity)
            PARSE_FIELD_DOUBLE(slowdown_distance)
            PARSE_FIELD_DOUBLE(stop_distance)
            PARSE_FIELD_DOUBLE(angular_kp)
            PARSE_FIELD_DOUBLE(angle_threshold)
            PARSE_FIELD_DOUBLE(lookahead_time)
            PARSE_FIELD_DOUBLE(boundary_margin)
        )
    };

    std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config);
    std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser);
    std::unique_ptr<NavigationConfiguration> load_navigation_from_toml(
        toml::table const &toml_data,
        std::vector<std::string> &parsed_sections);
} // namespace auto_battlebot
