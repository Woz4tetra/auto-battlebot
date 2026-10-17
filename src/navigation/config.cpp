#include "navigation/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_parser.hpp"
#include "navigation/fixed_velocity_navigation.hpp"
#include "navigation/motion_profile_navigation.hpp"
#include "navigation/noop_navigation.hpp"
#include "navigation/pursuit_navigation.hpp"
#include "navigation/rl_policy_navigation.hpp"
#include "robot_filter/config.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(NavigationConfiguration, NoopNavigationConfiguration, "NoopNavigation")
REGISTER_CONFIG(NavigationConfiguration, PursuitNavigationConfiguration, "PursuitNavigation")
REGISTER_CONFIG(NavigationConfiguration, MotionProfileNavigationConfiguration,
                "MotionProfileNavigation")
REGISTER_CONFIG(NavigationConfiguration, FixedVelocityNavigationConfiguration,
                "FixedVelocityNavigation")
REGISTER_CONFIG(NavigationConfiguration, RlPolicyNavigationConfiguration, "RlPolicyNavigation")

void RlPolicyNavigationConfiguration::apply_robot_filter(
    const RobotFilterConfiguration &robot_filter) {
    opponent_mode.reset();
    opponent_mode_source = "robot_filter type '" + robot_filter.type + "'";
    const auto *front_back = dynamic_cast<const RobotFrontBackFilterConfiguration *>(&robot_filter);
    if (front_back == nullptr || !front_back->motion_estimator) return;
    opponent_mode_source +=
        " with motion_estimator type '" + front_back->motion_estimator->type + "'";
    const auto *kalman = dynamic_cast<const KalmanMotionEstimatorConfiguration *>(
        front_back->motion_estimator.get());
    if (kalman != nullptr) opponent_mode = kalman->opponent_mode;
}

std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser) {
    return ConfigFactory<NavigationConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<NavigationConfiguration> load_navigation_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections,
    const PlantConfiguration &plant, const RobotFilterConfiguration &robot_filter) {
    auto section = toml_data["navigation"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [navigation]");
    }
    ConfigParser parser(*section, "navigation");
    auto config = parse_navigation_config(parser);
    config->apply_plant(plant);
    config->apply_robot_filter(robot_filter);
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
    if (config.type == "RlPolicyNavigation") {
        const auto &rl_config = dynamic_cast<const RlPolicyNavigationConfiguration &>(config);
        auto navigation = std::make_shared<RlPolicyNavigation>(rl_config, std::move(clock));
        // Loads and checks the bundle here, on the main thread at startup, so a bad bundle or a
        // HOLD opponent filter stops the app before the control loop starts. The control loop's
        // own initialize() calls then only reset.
        navigation->initialize();
        return navigation;
    }
    throw std::invalid_argument("Failed to load Navigation of type " + config.type);
}
}  // namespace auto_battlebot
