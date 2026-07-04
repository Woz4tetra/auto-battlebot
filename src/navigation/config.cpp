#include "navigation/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "navigation/fixed_velocity_navigation.hpp"
#include "navigation/motion_profile_navigation.hpp"
#include "navigation/noop_navigation.hpp"
#include "navigation/pursuit_navigation.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(NavigationConfiguration, NoopNavigationConfiguration, "NoopNavigation")
REGISTER_CONFIG(NavigationConfiguration, PursuitNavigationConfiguration, "PursuitNavigation")
REGISTER_CONFIG(NavigationConfiguration, MotionProfileNavigationConfiguration,
                "MotionProfileNavigation")
REGISTER_CONFIG(NavigationConfiguration, FixedVelocityNavigationConfiguration,
                "FixedVelocityNavigation")

std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser) {
    return ConfigFactory<NavigationConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<NavigationConfiguration> load_navigation_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["navigation"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [navigation]");
    }
    ConfigParser parser(*section, "navigation");
    auto config = parse_navigation_config(parser);
    parsed_sections.push_back("navigation");
    return config;
}

std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config,
                                                     std::shared_ptr<ClockInterface> clock) {
    spdlog::info("Selected {} for Navigation", config.type);
    if (config.type == "NoopNavigation") {
        return std::make_shared<NoopNavigation>();
    }
    if (config.type == "PursuitNavigation") {
        const auto &pursuit_config = dynamic_cast<const PursuitNavigationConfiguration &>(config);
        return std::make_shared<PursuitNavigation>(pursuit_config, std::move(clock));
    }
    if (config.type == "MotionProfileNavigation") {
        const auto &mp_config = dynamic_cast<const MotionProfileNavigationConfiguration &>(config);
        return std::make_shared<MotionProfileNavigation>(mp_config, std::move(clock));
    }
    if (config.type == "FixedVelocityNavigation") {
        const auto &fv_config = dynamic_cast<const FixedVelocityNavigationConfiguration &>(config);
        return std::make_shared<FixedVelocityNavigation>(fv_config);
    }
    throw std::invalid_argument("Failed to load Navigation of type " + config.type);
}
}  // namespace auto_battlebot
