#include "navigation/config.hpp"
#include "navigation/noop_navigation.hpp"
#include "navigation/pursuit_navigation.hpp"

namespace auto_battlebot
{
    // Automatic registration of config types
    REGISTER_CONFIG(NavigationConfiguration, NoopNavigationConfiguration, "NoopNavigation")
    REGISTER_CONFIG(NavigationConfiguration, PursuitNavigationConfiguration, "PursuitNavigation")

    std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser)
    {
        return ConfigFactory<NavigationConfiguration>::instance().create_and_parse(parser);
    }

    std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config)
    {
        std::cout << "Selected " + config.type + " for Navigation" << std::endl;
        if (config.type == "NoopNavigation")
        {
            return std::make_shared<NoopNavigation>();
        }
        if (config.type == "PursuitNavigation")
        {
            const auto &pursuit_config = dynamic_cast<const PursuitNavigationConfiguration &>(config);
            return std::make_shared<PursuitNavigation>(pursuit_config);
        }
        throw std::invalid_argument("Failed to load Navigation of type " + config.type);
    }
} // namespace auto_battlebot
