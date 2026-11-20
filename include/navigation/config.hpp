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

struct PursuitNavigationConfiguration : public NavigationConfiguration {
    double stop_distance_m = 0.0;
    double velocity_ramp_far_distance = 2.5;
    double velocity_ramp_near_distance = 1.0;
    double velocity_ramp_min_scale = 0.5;
    double brake_distance = 0.0;
    std::optional<double> wall_reverse_distance_m;
    double wall_reverse_min_speed = 1.0;
    double wall_heading_threshold_rad = M_PI / 3.0;
    double angular_kp = 3.0;
    double angular_kd = 0.0;
    double angle_threshold = 0.5;
    double lookahead_time = 0.1;
    double boundary_margin = 0.1;
    bool enable_hysteresis = true;
    std::optional<double> max_linear_command;
    std::optional<double> max_angular_command;
    bool hazard_tangent_enable = true;
    int hazard_tangent_max_iterations = 3;
    double hazard_waypoint_clearance_m = 0.06;
    double hazard_side_release_m = 0.05;
    double hazard_prediction_horizon_s = 0.25;

    double hazard_reverse_distance = 0.12;
    double hazard_heading_threshold = 1.047;
    double hazard_reverse_min_speed = 0.35;

    PursuitNavigationConfiguration() { type = "PursuitNavigation"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_RENAMED(stop_distance, stop_distance_m)
        PARSE_FIELD_DOUBLE(stop_distance_m)
        PARSE_FIELD_DOUBLE(velocity_ramp_far_distance)
        PARSE_FIELD_DOUBLE(velocity_ramp_near_distance)
        PARSE_FIELD_DOUBLE(velocity_ramp_min_scale)
        PARSE_FIELD_DOUBLE(brake_distance)
        PARSE_RENAMED(wall_reverse_distance, wall_reverse_distance_m)
        PARSE_FIELD_NULLABLE_DOUBLE(wall_reverse_distance_m)
        PARSE_FIELD_DOUBLE(wall_reverse_min_speed)
        PARSE_RENAMED(wall_heading_threshold, wall_heading_threshold_rad)
        PARSE_FIELD_DOUBLE(wall_heading_threshold_rad)
        PARSE_FIELD_DOUBLE(angular_kp)
        PARSE_FIELD_DOUBLE(angular_kd)
        PARSE_FIELD_DOUBLE(angle_threshold)
        PARSE_FIELD_DOUBLE(lookahead_time)
        PARSE_FIELD_DOUBLE(boundary_margin)
        PARSE_FIELD_BOOL(enable_hysteresis)
        PARSE_FIELD_NULLABLE_DOUBLE(max_linear_command)
        PARSE_FIELD_NULLABLE_DOUBLE(max_angular_command)
        PARSE_FIELD_BOOL(hazard_tangent_enable)
        PARSE_FIELD(hazard_tangent_max_iterations)
        PARSE_FIELD_DOUBLE(hazard_waypoint_clearance_m)
        PARSE_FIELD_DOUBLE(hazard_side_release_m)
        PARSE_FIELD_DOUBLE(hazard_prediction_horizon_s)
        PARSE_FIELD_DOUBLE(hazard_reverse_distance)
        PARSE_FIELD_DOUBLE(hazard_heading_threshold)
        PARSE_FIELD_DOUBLE(hazard_reverse_min_speed)
    )
    // clang-format on
};

struct FixedVelocityNavigationConfiguration : public NavigationConfiguration {
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;

    FixedVelocityNavigationConfiguration() { type = "FixedVelocityNavigation"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(linear_x)
        PARSE_FIELD_DOUBLE(linear_y)
        PARSE_FIELD_DOUBLE(angular_z)
    )
    // clang-format on
};

struct MotionProfileNavigationConfiguration : public NavigationConfiguration {
    JigPlantParams plant;
    double deadzone = 0.0;
    double accel_limit = 8.5;

    double steer_brake_floor = 0.3;
    double attack_terminal_speed_fraction = 1.0;
    double run_away_terminal_speed_fraction = 0.0;
    double stop_distance_m = 0.15;
    double speed_kp = 0.15;
    double speed_ki = 0.0;

    bool compensate_angular_droop = false;

    double angular_droop_floor = 0.3;
    double angular_kp = 12.7;
    double angular_kd = 0.32;
    double angle_threshold = 1.7;
    double max_yaw_rate = 7.93;
    std::optional<double> max_angular_command = 1.0;
    bool enable_hysteresis = true;
    double boundary_margin = 0.1;
    std::optional<double> max_linear_command = 1.0;
    std::optional<double> wall_reverse_distance_m;
    double wall_reverse_min_speed = 0.0;
    double wall_heading_threshold_rad = 0.6;
    bool hazard_tangent_enable = true;
    int hazard_tangent_max_iterations = 3;
    double hazard_waypoint_clearance_m = 0.06;
    double hazard_side_release_m = 0.05;
    bool hazard_barrier_enable = true;
    double hazard_prediction_horizon_s = 0.25;

    MotionProfileNavigationConfiguration() { type = "MotionProfileNavigation"; }

    void apply_plant(const PlantConfiguration &plant_config) override {
        if (!plant_config.params.has_value()) {
            throw ConfigValidationError(
                "navigation type = 'MotionProfileNavigation' requires the top-level [plant] table; "
                "see "
                "docs/reference/control/plant_backed_control.md");
        }
        plant = *plant_config.params;
    }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(deadzone)
        PARSE_FIELD_DOUBLE(accel_limit)
        PARSE_FIELD_DOUBLE(steer_brake_floor)
        PARSE_FIELD_DOUBLE(attack_terminal_speed_fraction)
        PARSE_FIELD_DOUBLE(run_away_terminal_speed_fraction)
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
        PARSE_FIELD_NULLABLE_DOUBLE(max_angular_command)
        PARSE_FIELD_BOOL(enable_hysteresis)
        PARSE_FIELD_DOUBLE(boundary_margin)
        PARSE_FIELD_NULLABLE_DOUBLE(max_linear_command)
        PARSE_RENAMED(wall_reverse_distance, wall_reverse_distance_m)
        PARSE_FIELD_NULLABLE_DOUBLE(wall_reverse_distance_m)
        PARSE_FIELD_DOUBLE(wall_reverse_min_speed)
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
    std::optional<OpponentMode> opponent_mode;
    std::string opponent_mode_source = "no [robot_filter] config was applied";
    double max_our_unmeasured_s = 0.15;

    bool hazard_barrier_enable = false;
    double hazard_prediction_horizon_s = 0.25;
    std::optional<JigPlantParams> plant;

    RlPolicyNavigationConfiguration() { type = "RlPolicyNavigation"; }

    void apply_plant(const PlantConfiguration &plant_config) override {
        plant = plant_config.params;
        if (hazard_barrier_enable && !plant.has_value()) {
            throw ConfigValidationError(
                "type = 'RlPolicyNavigation' with hazard_barrier_enable = true requires the "
                "top-level [plant] table; see docs/reference/control/plant_backed_control.md");
        }
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
        PARSE_FIELD_DOUBLE(max_our_unmeasured_s)
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
