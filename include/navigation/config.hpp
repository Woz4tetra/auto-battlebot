#pragma once

#include "data_structures.hpp"
#include "navigation/navigation_interface.hpp"
#include "config_factory.hpp"
#include "config_parser.hpp"

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

    std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config);
    std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser);
} // namespace auto_battlebot
