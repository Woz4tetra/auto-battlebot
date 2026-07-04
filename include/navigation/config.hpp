#pragma once

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "navigation/navigation_interface.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct NavigationConfiguration {
    std::string type;
    virtual ~NavigationConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopNavigationConfiguration : public NavigationConfiguration {
    NoopNavigationConfiguration() { type = "NoopNavigation"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct PursuitNavigationConfiguration : public NavigationConfiguration {
    /** Distance at which to stop in meters */
    double stop_distance = 0.0;

    /** Distance (m) at which velocity ramp begins. At this distance and beyond, speed is
     * velocity_ramp_min_scale * max. */
    double velocity_ramp_far_distance = 2.5;

    /** Distance (m) at which velocity reaches maximum. At this distance and below, speed is max. */
    double velocity_ramp_near_distance = 1.0;

    /** Speed scale applied at distances >= velocity_ramp_far_distance (0..1, fraction of max). */
    double velocity_ramp_min_scale = 0.5;

    /** Coast-aware brake: within this distance (m) of the target, the commanded speed ramps
     * linearly down to zero so the robot decelerates into the goal instead of driving in at full
     * speed and coasting past it. Should cover the stopping lead (latency travel + coast distance).
     * 0 disables (ram behavior: full speed into contact). */
    double brake_distance = 0.0;

    /** Distance from any field wall at which linear_x is reversed to back away (m).
     *  0 = disabled. */
    double wall_reverse_distance = 0.0;

    /** Minimum speed (m/s) used when wall reverse engages and no forward drive was commanded.
     *  Ensures a meaningful reversal even when the robot was stationary or turning in place. */
    double wall_reverse_min_speed = 1.0;

    /** Heading must be within this angle of the nearest wall normal to trigger reversal (rad).
     *  Default ~60 deg. */
    double wall_heading_threshold = M_PI / 3.0;

    /** Proportional gain for angular control */
    double angular_kp = 3.0;

    /** Derivative gain for angular control (0 = P-only) */
    double angular_kd = 0.0;

    /** Angle threshold below which robot drives forward (rad) */
    double angle_threshold = 0.5;

    /** Lookahead time for target position prediction in seconds */
    double lookahead_time = 0.1;

    /** Minimum distance to maintain from field boundaries in meters */
    double boundary_margin = 0.1;

    /** Enable hysteresis to prevent dithering when target is behind */
    bool enable_hysteresis = true;

    /** Cap on the magnitude of the normalized forward (linear_x) command (0..1, a fraction of full
     * drive; the plant/transmitter scales it by max_linear_speed). 0 = no limit. */
    double max_linear_command = 0.0;

    /** Cap on the magnitude of the normalized turn (angular_z) command (0..1, a fraction of full
     * turn; the plant/transmitter scales it by max_angular_speed). 0 = no limit. */
    double max_angular_command = 0.0;

    PursuitNavigationConfiguration() { type = "PursuitNavigation"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(stop_distance)
        PARSE_FIELD_DOUBLE(velocity_ramp_far_distance)
        PARSE_FIELD_DOUBLE(velocity_ramp_near_distance)
        PARSE_FIELD_DOUBLE(velocity_ramp_min_scale)
        PARSE_FIELD_DOUBLE(brake_distance)
        PARSE_FIELD_DOUBLE(wall_reverse_distance)
        PARSE_FIELD_DOUBLE(wall_reverse_min_speed)
        PARSE_FIELD_DOUBLE(wall_heading_threshold)
        PARSE_FIELD_DOUBLE(angular_kp)
        PARSE_FIELD_DOUBLE(angular_kd)
        PARSE_FIELD_DOUBLE(angle_threshold)
        PARSE_FIELD_DOUBLE(lookahead_time)
        PARSE_FIELD_DOUBLE(boundary_margin)
        PARSE_FIELD_BOOL(enable_hysteresis)
        PARSE_FIELD_DOUBLE(max_linear_command)
        PARSE_FIELD_DOUBLE(max_angular_command)
    )
    // clang-format on
};

struct FixedVelocityNavigationConfiguration : public NavigationConfiguration {
    /** Fixed linear X command in m/s */
    double linear_x = 0.0;

    /** Fixed linear Y command in m/s */
    double linear_y = 0.0;

    /** Fixed angular Z command in rad/s */
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
    // --- Plant parameters (measured, per-robot; see mr_stabs_mk2_calibration.md) ---

    /** Max forward speed (m/s). Plant/transmitter scales the normalized command by this, so it is
     * also the divisor that maps a reference speed (m/s) back to a normalized command. */
    double max_linear_speed_fwd = 5.6;

    /** Max reverse speed (m/s); reverse drive is a weaker brake than forward drive. */
    double max_linear_speed_rev = 4.84;

    /** Accel (spin-up) time constant (s); used for the inverse-plant feedforward of dv/dt so a
     * rising reference commands extra thrust to overcome the drivetrain lag. */
    double tau_accel = 0.058;

    /** Coast (decel) time constant (s). With the actuation latency this sets the brake horizon:
     * a first-order plant's residual travel after commanding v_term is ~ v*(tau_decel+latency), so
     * the reference speed is capped at (d - margin)/(tau_decel+latency) to stop on the goal. */
    double tau_decel = 0.078;

    /** Actuation latency (s): commands bite this late, adding v*latency of travel to the brake
     * horizon above. */
    double latency = 0.06;

    /** Residual command deadzone (0..1 fraction) the plant still shows. Exact inverse-deadzone is
     * applied to the final command when > 0. Defaults 0 because the transmitter's
     * lifted_deadzone_percent handles the physical deadzone upstream on the real robot (and the
     * kinematic sim's residual deadzone is 0). */
    double deadzone = 0.0;

    /** Rate limit (m/s^2) on how fast the reference speed may rise, for a clean launch and a
     * bounded feedforward derivative. Braking (falling reference) is never rate-limited. */
    double accel_limit = 30.0;

    // --- Trajectory ---

    /** Commanded terminal speed at the goal (m/s). 0 = precise zero-velocity stop; > 0 = ram,
     * arrive at the goal at this contact speed and drive through. */
    double terminal_velocity = 0.0;

    /** Distance (m) at which a zero-velocity mission is complete and the command is cut. Only used
     * when terminal_velocity == 0 (ram never stops). */
    double stop_distance = 0.15;

    // --- Speed feedback (closes the loop on plant-model error) ---

    /** Proportional gain mapping speed error (v_ref - v_actual, m/s) to a normalized command. */
    double speed_kp = 0.15;

    /** Integral gain on speed error. 0 = P-only. */
    double speed_ki = 0.0;

    // --- Angular control (same semantics as PursuitNavigation) ---

    /** Proportional gain angular control (rad/s per rad error). */
    double angular_kp = 0.4;

    /** Derivative gain angular control (0 = P-only). */
    double angular_kd = 0.01;

    /** Heading error (rad) below which the robot drives forward; above it, turn in place. */
    double angle_threshold = 1.7;

    /** Cap on magnitude of the normalized turn (angular_z) command (0..1). 0 = no limit. */
    double max_angular_command = 0.25;

    /** Enable turn-direction hysteresis to prevent dithering when the target is behind. */
    bool enable_hysteresis = true;

    // --- Misc / safety ---

    /** Minimum distance maintained from field boundaries (m). */
    double boundary_margin = 0.1;

    /** Cap on magnitude of the normalized forward (linear_x) command (0..1). 0 = no limit. */
    double max_linear_command = 1.0;

    /** Distance to any wall (m) at which linear_x is reversed to back away. 0 = disabled. */
    double wall_reverse_distance = 0.0;

    /** Minimum reverse speed (normalized) used when wall reverse engages. */
    double wall_reverse_min_speed = 0.0;

    /** Only reverse off a wall when heading is within this angle (rad) of facing it. */
    double wall_heading_threshold = 0.6;

    MotionProfileNavigationConfiguration() { type = "MotionProfileNavigation"; }

    // clang-format off
    PARSE_CONFIG_FIELDS(
        PARSE_FIELD_DOUBLE(max_linear_speed_fwd)
        PARSE_FIELD_DOUBLE(max_linear_speed_rev)
        PARSE_FIELD_DOUBLE(tau_accel)
        PARSE_FIELD_DOUBLE(tau_decel)
        PARSE_FIELD_DOUBLE(latency)
        PARSE_FIELD_DOUBLE(deadzone)
        PARSE_FIELD_DOUBLE(accel_limit)
        PARSE_FIELD_DOUBLE(terminal_velocity)
        PARSE_FIELD_DOUBLE(stop_distance)
        PARSE_FIELD_DOUBLE(speed_kp)
        PARSE_FIELD_DOUBLE(speed_ki)
        PARSE_FIELD_DOUBLE(angular_kp)
        PARSE_FIELD_DOUBLE(angular_kd)
        PARSE_FIELD_DOUBLE(angle_threshold)
        PARSE_FIELD_DOUBLE(max_angular_command)
        PARSE_FIELD_BOOL(enable_hysteresis)
        PARSE_FIELD_DOUBLE(boundary_margin)
        PARSE_FIELD_DOUBLE(max_linear_command)
        PARSE_FIELD_DOUBLE(wall_reverse_distance)
        PARSE_FIELD_DOUBLE(wall_reverse_min_speed)
        PARSE_FIELD_DOUBLE(wall_heading_threshold)
    )
    // clang-format on
};

std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config,
                                                     std::shared_ptr<ClockInterface> clock);
std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser);
std::unique_ptr<NavigationConfiguration> load_navigation_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
