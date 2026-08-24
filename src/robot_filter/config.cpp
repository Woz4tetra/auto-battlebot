#include "robot_filter/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_cast.hpp"
#include "config/config_parser.hpp"
#include "robot_filter/dead_reckoning_motion_estimator.hpp"
#include "robot_filter/ground_truth_robot_filter.hpp"
#include "robot_filter/kalman_motion_estimator.hpp"
#include "robot_filter/noop_robot_filter.hpp"
#include "robot_filter/robot_front_back_filter.hpp"

namespace auto_battlebot {
// Automatic registration of config types
REGISTER_CONFIG(RobotFilterConfiguration, NoopRobotFilterConfiguration, "NoopRobotFilter")
REGISTER_CONFIG(RobotFilterConfiguration, GroundTruthRobotFilterConfiguration,
                "GroundTruthRobotFilter")
REGISTER_CONFIG(RobotFilterConfiguration, RobotFrontBackFilterConfiguration, "RobotFrontBackFilter")
REGISTER_CONFIG(MotionEstimatorConfiguration, DeadReckoningMotionEstimatorConfiguration,
                "DeadReckoningMotionEstimator")
REGISTER_CONFIG(MotionEstimatorConfiguration, KalmanMotionEstimatorConfiguration,
                "KalmanMotionEstimator")

std::unique_ptr<MotionEstimatorInterface> make_motion_estimator(
    const MotionEstimatorConfiguration &config) {
    if (config.type == "DeadReckoningMotionEstimator") {
        return std::make_unique<DeadReckoningMotionEstimator>(
            config_cast<DeadReckoningMotionEstimatorConfiguration>(config));
    } else if (config.type == "KalmanMotionEstimator") {
        return std::make_unique<KalmanMotionEstimator>(
            config_cast<KalmanMotionEstimatorConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load motion estimator of type " + config.type);
}

std::unique_ptr<RobotFilterConfiguration> parse_robot_filter_config(ConfigParser &parser) {
    return ConfigFactory<RobotFilterConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<RobotFilterConfiguration> load_robot_filter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections,
    const PlantConfiguration &plant) {
    auto section = toml_data["robot_filter"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [robot_filter]");
    }
    ConfigParser parser(*section, "robot_filter");
    auto config = parse_robot_filter_config(parser);
    config->apply_plant(plant);
    parsed_sections.push_back("robot_filter");
    return config;
}

std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config,
                                                        std::shared_ptr<ClockInterface> clock) {
    spdlog::info("Selected {} for RobotFilter", config.type);
    if (config.type == "NoopRobotFilter") {
        return std::make_shared<NoopRobotFilter>();
    } else if (config.type == "GroundTruthRobotFilter") {
        return std::make_shared<GroundTruthRobotFilter>(std::move(clock));
    } else if (config.type == "RobotFrontBackFilter") {
        return std::make_shared<RobotFrontBackFilter>(
            config_cast<RobotFrontBackFilterConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load RobotFilter of type " + config.type);
}
}  // namespace auto_battlebot
