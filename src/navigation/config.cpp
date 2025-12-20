#include "navigation/config.hpp"
#include "navigation/noop_navigation.hpp"
#include "config_parser.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(NavigationConfiguration, NoopNavigationConfiguration, "NoopNavigation")

    std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser)
    {
        return ConfigFactory<NavigationConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config)
    {
        if (config.type == "NoopNavigation")
        {
            return std::make_shared<NoopNavigation>();
        }
        throw std::invalid_argument("Failed to load Navigation of type " + config.type);
    }
} // namespace auto_battlebot
