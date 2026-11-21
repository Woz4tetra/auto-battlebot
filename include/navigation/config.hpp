#pragma once

#include <optional>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "navigation/navigation_interface.hpp"
#include "plant/config.hpp"
#include "rl_policy/config.hpp"
#include "robot_filter/motion_estimator_config.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct RobotFilterConfiguration;

struct NavigationConfiguration {
    std::string type;
    virtual ~NavigationConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}

    virtual void apply_plant([[maybe_unused]] const PlantConfiguration &plant) {}

    virtual void apply_robot_filter([[maybe_unused]] const RobotFilterConfiguration &robot_filter) {
    }
};

struct NoopNavigationConfiguration : public NavigationConfiguration {
    NoopNavigationConfiguration() { type = "NoopNavigation"; }

    PARSE_CONFIG_FIELDS()
};

struct MotionProfileNavigationConfiguration : public NavigationConfiguration {
    JigPlantParams plant;
    double accel_limit = 8.5;

    double steer_brake_floor = 0.3;
    double attack_terminal_speed_mps = 4.35;
    double run_away_terminal_speed_mps = 0.0;
    double stop_distance_m = 0.15;
    double speed_kp = 0.65;
    double speed_ki = 0.0;

    bool compensate_angular_droop = false;

    double angular_droop_floor = 0.3;
    double angular_kp = 12.7;
    double angular_kd = 0.32;
    double angle_threshold = 1.7;
    double max_yaw_rate = 7.93;
    bool enable_hysteresis = true;
    double boundary_margin = 0.1;
    int hazard_push_out_max_passes = 6;
    std::optional<double> max_linear_speed_mps;
    std::optional<double> wall_reverse_distance_m;
    double wall_reverse_min_speed_mps = 0.0;
    double wall_heading_threshold_rad = 0.6;
    bool hazard_tangent_enable = true;
    int hazard_tangent_max_iterations = 3;
    double hazard_waypoint_clearance_m = 0.06;
    double hazard_side_release_m = 0.05;
    bool hazard_barrier_enable = true;
    double hazard_prediction_horizon_s = 0.25;

    MotionProfileNavigationConfiguration() { type = "MotionProfileNavigation"; }

    void apply_plant(const PlantConfiguration &plant_config) override {
        plant = plant_config.params;
    }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_RENAMED(deadzone, command_shaper.linear_deadzone)
        PARSE_FIELD_DOUBLE(accel_limit)
        PARSE_FIELD_DOUBLE(steer_brake_floor)
        PARSE_RENAMED(attack_terminal_speed_fraction, attack_terminal_speed_mps)
        PARSE_FIELD_DOUBLE(attack_terminal_speed_mps)
        PARSE_RENAMED(run_away_terminal_speed_fraction, run_away_terminal_speed_mps)
        PARSE_FIELD_DOUBLE(run_away_terminal_speed_mps)
        PARSE_RENAMED(stop_distance, stop_distance_m)
        PARSE_FIELD_DOUBLE(stop_distance_m)
        PARSE_FIELD_DOUBLE(speed_kp)
        PARSE_FIELD_DOUBLE(speed_ki)
        PARSE_FIELD_BOOL(compensate_angular_droop)
        PARSE_FIELD_DOUBLE(angular_droop_floor)
        PARSE_FIELD_DOUBLE(angular_kp)
        PARSE_FIELD_DOUBLE(angular_kd)
        PARSE_FIELD_DOUBLE(angle_threshold)
        PARSE_FIELD_DOUBLE(max_yaw_rate)
        PARSE_FIELD_BOOL(enable_hysteresis)
        PARSE_FIELD_DOUBLE(boundary_margin)
        PARSE_FIELD(hazard_push_out_max_passes)
        PARSE_RENAMED(max_linear_command, max_linear_speed_mps)
        PARSE_FIELD_NULLABLE_DOUBLE(max_linear_speed_mps)
        PARSE_RENAMED(wall_reverse_distance, wall_reverse_distance_m)
        PARSE_FIELD_NULLABLE_DOUBLE(wall_reverse_distance_m)
        PARSE_RENAMED(wall_reverse_min_speed, wall_reverse_min_speed_mps)
        PARSE_FIELD_DOUBLE(wall_reverse_min_speed_mps)
        PARSE_RENAMED(wall_heading_threshold, wall_heading_threshold_rad)
        PARSE_FIELD_DOUBLE(wall_heading_threshold_rad)
        PARSE_FIELD_BOOL(hazard_tangent_enable)
        PARSE_FIELD(hazard_tangent_max_iterations)
        PARSE_FIELD_DOUBLE(hazard_waypoint_clearance_m)
        PARSE_FIELD_DOUBLE(hazard_side_release_m)
        PARSE_FIELD_BOOL(hazard_barrier_enable)
        PARSE_FIELD_DOUBLE(hazard_prediction_horizon_s)
    )
    // clang-format on
};

struct RlPolicyNavigationConfiguration : public NavigationConfiguration {
    rl_policy::RlPolicyConfiguration policy;
    bool has_opponent_filter = false;
    std::string opponent_filter_source = "no [robot_filter] config was applied";

    bool hazard_barrier_enable = false;
    double hazard_prediction_horizon_s = 0.25;
    JigPlantParams plant;

    RlPolicyNavigationConfiguration() { type = "RlPolicyNavigation"; }

    void apply_plant(const PlantConfiguration &plant_config) override {
        plant = plant_config.params;
    }

    void parse_fields(ConfigParser &parser) override {
        const toml::table *table = parser.get_table("policy");
        if (table == nullptr) {
            throw ConfigValidationError(
                "navigation type = 'RlPolicyNavigation' requires a [navigation.policy] table "
                "naming the model bundle");
        }
        ConfigParser policy_parser(*table, "navigation.policy");
        policy.parse(policy_parser);
        PARSE_FIELD_BOOL(hazard_barrier_enable)
        PARSE_FIELD_DOUBLE(hazard_prediction_horizon_s)
    }

    void apply_robot_filter(const RobotFilterConfiguration &robot_filter) override;
};

std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config,
                                                     std::shared_ptr<ClockInterface> clock);
std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser);
std::unique_ptr<NavigationConfiguration> load_navigation_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections,
    const PlantConfiguration &plant, const RobotFilterConfiguration &robot_filter);
}  // namespace auto_battlebot
