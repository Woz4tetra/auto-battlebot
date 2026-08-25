#pragma once

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "navigation/navigation_interface.hpp"
#include "plant/config.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct NavigationConfiguration {
    std::string type;
    virtual ~NavigationConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}

    /**
     * Hand over the shared [plant] table, after parse_fields. Separate from parsing because the
     * plant is a top-level section: this config's own parser never sees it. Controllers that do
     * not model the plant (pursuit, fixed velocity) keep the no-op.
     */
    virtual void apply_plant([[maybe_unused]] const PlantConfiguration &plant) {}
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

    // --- Hazard avoidance ---
    //
    // Options 1, 2 and 4 only. The speed cap (option 3) is stated in m/s against a measured
    // yaw rate, and PursuitNavigation closes its heading loop in normalized command against an
    // unfitted plant, so there is no honest number to cap with here. It lives in
    // MotionProfileNavigation, which has the fit. Options 1 and 2 are the floor and ceiling:
    // keeping the goal legal, and a last-ditch reverse. Options 3 and 4 do the actual work, and are
    // one behaviour -- "cap the speed enough to make the turn, then take the turn" -- so tuning
    // either alone will mislead. See docs/hazard_avoidance_plan.md.

    /** Steer at a tangent point on a blocking hazard's rim instead of at the true goal
     * (option 4, the primary response). Costs nothing in navigation: both controllers only ever
     * chase a Pose2D. */
    bool hazard_tangent_enable = true;

    /** How many times to re-test the substituted waypoint against the remaining hazards. A
     * greedy one-hazard-at-a-time step, not a planner: a field holds a handful of hazards, not a
     * maze. When it does not converge the speed cap and the reverse backstop cover it. */
    int hazard_tangent_max_iterations = 3;

    /** Clearance (m) the tangent waypoint leaves beyond the keep-out radius. Aiming at the
     * keep-out rim exactly means arriving with zero clearance and grinding along the edge; a
     * little extra makes the run from the waypoint to the goal unblocked, so the true goal comes
     * back on its own instead of the robot orbiting. */
    double hazard_waypoint_clearance_m = 0.06;

    /** Release the latched pass side once the direct run to the goal clears the hazard by this
     * much (m). Latching is the same pattern as the turn-direction hysteresis: without it the
     * robot re-picks a side every tick and cuts back across the hazard it was rounding. */
    double hazard_side_release_m = 0.05;

    /** Seconds ahead to advance a TRACKED hazard along its filtered velocity before testing it.
     * The house bot moving across our path is the case this exists for: reacting to where it is
     * now aims at where it was. */
    double hazard_prediction_horizon_s = 0.25;

    /** Distance (m) from a hazard edge inside which linear_x is overridden to reverse, when the
     * heading also points at it. 0 = disabled.
     *
     * This is a stopping response, and the numbers say it will fire too late to save a
     * full-speed approach: it is a backstop for what the steering layer cannot solve, mainly a
     * hazard that appears close because the house bot drove into our path or its track
     * initialised late. If a sweep shows it firing often, that is evidence options 3 and 4 are
     * mistuned, not evidence this is working. */
    double hazard_reverse_distance = 0.12;

    /** Heading must point within this angle (rad) of the hazard for the reverse to trigger. */
    double hazard_heading_threshold = 1.047;

    /** Reverse speed floor while the hazard backstop is active (normalized command). */
    double hazard_reverse_min_speed = 0.35;

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
    // --- Plant parameters ---
    //
    // The measured drivetrain, stamped in by apply_plant from the shared top-level [plant] table
    // rather than parsed from [navigation]. The controller inverts the same fit the our-robot EKF
    // propagates, so both read one table: k_fwd / k_rev are the divisors mapping a reference speed
    // back to a normalized command, tau_lin_a feedforwards dv/dt against spin-up lag, tau_lin_d
    // and delay_s set the brake horizon, c_sb is the steer-brake loss the controller divides back
    // out, and dz_ang_l / dz_ang_r turn a commanded turn into the effective turn the coupling
    // terms are defined against.
    JigPlantParams plant;

    /** Residual command deadzone (0..1 fraction) the plant still shows. Exact inverse-deadzone is
     * applied to the final command when > 0. Distinct from the fit's dz_lin_fwd / dz_lin_rev, and
     * defaults 0 because the transmitter's lifted_deadzone_percent handles the physical deadzone
     * upstream on the real robot (and the kinematic sim's residual deadzone is 0). */
    double deadzone = 0.0;

    /** Rate limit (m/s^2) on how fast the reference speed may rise, for a clean launch and a
     * bounded feedforward derivative. Braking (falling reference) is never rate-limited. */
    double accel_limit = 8.5;

    /** Floor on the steer-brake authority multiplier. 1/c_sb = 0.370 is where the fitted linear
     * loss reaches zero and where the fit's own residuals say the shape stops describing the
     * plant, so the compensation saturates here instead of extrapolating into a singularity. */
    double steer_brake_floor = 0.3;

    // --- Trajectory ---

    // Commanded terminal speed at the goal, one per behavior mode: the driver's switch decides
    // whether the mission is to hit something or to get away from it, and those want opposite
    // arrivals. Both are a fraction of the plant's k_fwd, clamped to [0, 1]: 0 is a precise
    // zero-velocity stop and 1 is full speed. Normalized rather than m/s so a refit rescales
    // them instead of leaving a hand-copied speed stale.

    /** Terminal speed while the driver has ATTACK selected, as a fraction of the plant's k_fwd. */
    double attack_terminal_speed_fraction = 1.0;

    /** Terminal speed while the driver has RUN_AWAY selected, as a fraction of the plant's
     * k_fwd. */
    double run_away_terminal_speed_fraction = 0.0;

    /** Distance (m) at which a zero-velocity mission is complete and the command is cut. Only used
     * when the active terminal velocity is 0 (a drive-through mission never stops). */
    double stop_distance = 0.15;

    // --- Speed feedback (closes the loop on plant-model error) ---

    /** Proportional gain mapping speed error (v_ref - v_actual, m/s) to a normalized command. */
    double speed_kp = 0.15;

    /** Integral gain on speed error. 0 = P-only. */
    double speed_ki = 0.0;

    // --- Angular control ---
    //
    // Closed in rad/s, the same shape as the linear channel: a PD sets a yaw-rate reference, an
    // inverse-plant feedforward turns it into a normalized command through the plant's k_ang and
    // the regime-appropriate tau_ang_a / tau_ang_d. PursuitNavigation's angular gains are NOT
    // interchangeable with these; there they are normalized command per radian, here they are
    // rad/s per radian, a factor of k_ang apart.

    /** Compensate the plant's angular droop, the fit's c_ad = 0.463: the plant multiplies yaw
     * authority by (1 - c_ad*|u_lin_eff|), so the heading loop is weaker at speed than in place.
     *
     * Defaults OFF even though the coefficient is measured, because compensating it makes the
     * controller worse. On the 90-degree turning approach, enabling it takes terminal error from
     * 0.008 m to 0.067 m and time-to-goal from 1.10 s to 2.83 s. Compensating buys faster heading
     * convergence by commanding a harder turn, and a harder turn is exactly what the steer-brake
     * term charges forward speed for. The heading loop is closed-loop already and gets there
     * without the help; the forward speed it spends is not refunded. */
    bool compensate_angular_droop = false;

    /** Floor on the droop multiplier. c_ad = 0.463 never reaches zero, so this is a guard against
     * a refit pushing the coefficient past 1, not a shape limit like steer_brake_floor. */
    double angular_droop_floor = 0.3;

    /** Proportional gain on heading error (rad/s per rad). */
    double angular_kp = 12.7;

    /** Derivative gain on heading error, rad/s per rad/s (0 = P-only). */
    double angular_kd = 0.32;

    /** Heading error (rad) below which the robot drives forward; above it, turn in place. */
    double angle_threshold = 1.7;

    /** Cap on the yaw-rate reference (rad/s). 0 = no limit. Bounds rotation per control tick: at
     * 30 Hz, 7.9 rad/s is 15 degrees per tick. */
    double max_yaw_rate = 7.93;

    /** Cap on magnitude of the normalized turn (angular_z) command (0..1). 0 = no limit. This is
     * the output-side safety clamp; max_yaw_rate is the one that shapes behaviour. */
    double max_angular_command = 1.0;

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

    // --- Hazard avoidance ---
    //
    // Four layers, weakest to strongest. Options 1 and 2 are the floor and ceiling: keeping the
    // goal legal, and a last-ditch reverse. Options 3 and 4 do the actual work, and are one
    // behaviour -- "cap the speed enough to make the turn, then take the turn" -- so tuning
    // either alone will mislead. See docs/hazard_avoidance_plan.md.

    /** Steer at a tangent point on a blocking hazard's rim instead of at the true goal
     * (option 4, the primary response). Costs nothing in navigation: both controllers only ever
     * chase a Pose2D. */
    bool hazard_tangent_enable = true;

    /** How many times to re-test the substituted waypoint against the remaining hazards. A
     * greedy one-hazard-at-a-time step, not a planner: a field holds a handful of hazards, not a
     * maze. When it does not converge the speed cap and the reverse backstop cover it. */
    int hazard_tangent_max_iterations = 3;

    /** Clearance (m) the tangent waypoint leaves beyond the keep-out radius. Aiming at the
     * keep-out rim exactly means arriving with zero clearance and grinding along the edge; a
     * little extra makes the run from the waypoint to the goal unblocked, so the true goal comes
     * back on its own instead of the robot orbiting. */
    double hazard_waypoint_clearance_m = 0.06;

    /** Release the latched pass side once the direct run to the goal clears the hazard by this
     * much (m). Latching is the same pattern as the turn-direction hysteresis: without it the
     * robot re-picks a side every tick and cuts back across the hazard it was rounding. */
    double hazard_side_release_m = 0.05;

    /** Cap forward speed to what the achievable turn radius can clear (option 3). 0 = off. The
     * cap is `max_yaw_rate * L^2 / (2 R)` at along-track range L, so it stays out of the way
     * until a hazard is genuinely close and near the heading. */
    bool hazard_speed_cap_enable = true;

    /** Floor (m/s) under the speed cap. A cap that reaches zero recreates the stop-in-time
     * behaviour this plant cannot deliver and strands the robot with the hazard in front of
     * it. */
    double hazard_speed_cap_floor = 0.35;

    /** Seconds ahead to advance a TRACKED hazard along its filtered velocity before testing it.
     * The house bot moving across our path is the case this exists for: reacting to where it is
     * now aims at where it was. */
    double hazard_prediction_horizon_s = 0.25;

    /** Distance (m) from a hazard edge inside which linear_x is overridden to reverse, when the
     * heading also points at it. 0 = disabled.
     *
     * This is a stopping response, and the numbers say it will fire too late to save a
     * full-speed approach: it is a backstop for what the steering layer cannot solve, mainly a
     * hazard that appears close because the house bot drove into our path or its track
     * initialised late. If a sweep shows it firing often, that is evidence options 3 and 4 are
     * mistuned, not evidence this is working. */
    double hazard_reverse_distance = 0.12;

    /** Heading must point within this angle (rad) of the hazard for the reverse to trigger. */
    double hazard_heading_threshold = 1.047;

    /** Reverse speed floor while the hazard backstop is active (normalized command). */
    double hazard_reverse_min_speed = 0.35;

    MotionProfileNavigationConfiguration() { type = "MotionProfileNavigation"; }

    /**
     * Every plant term this controller inverts comes from the fit, so there is nothing sensible
     * to fall back on when the table is missing: a defaulted plant would brake and feedforward
     * against a drivetrain no experiment measured.
     */
    void apply_plant(const PlantConfiguration &plant_config) override {
        if (!plant_config.params.has_value()) {
            throw ConfigValidationError(
                "navigation type = 'MotionProfileNavigation' requires the top-level [plant] "
                "table: its brake schedule and inverse-plant feedforward are derived from the "
                "jig fit and have no defaults. See docs/plant_backed_control.md");
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
        PARSE_FIELD_DOUBLE(stop_distance)
        PARSE_FIELD_DOUBLE(speed_kp)
        PARSE_FIELD_DOUBLE(speed_ki)
        PARSE_FIELD_BOOL(compensate_angular_droop)
        PARSE_FIELD_DOUBLE(angular_droop_floor)
        PARSE_FIELD_DOUBLE(angular_kp)
        PARSE_FIELD_DOUBLE(angular_kd)
        PARSE_FIELD_DOUBLE(angle_threshold)
        PARSE_FIELD_DOUBLE(max_yaw_rate)
        PARSE_FIELD_DOUBLE(max_angular_command)
        PARSE_FIELD_BOOL(enable_hysteresis)
        PARSE_FIELD_DOUBLE(boundary_margin)
        PARSE_FIELD_DOUBLE(max_linear_command)
        PARSE_FIELD_DOUBLE(wall_reverse_distance)
        PARSE_FIELD_DOUBLE(wall_reverse_min_speed)
        PARSE_FIELD_DOUBLE(wall_heading_threshold)
        PARSE_FIELD_BOOL(hazard_tangent_enable)
        PARSE_FIELD(hazard_tangent_max_iterations)
        PARSE_FIELD_DOUBLE(hazard_waypoint_clearance_m)
        PARSE_FIELD_DOUBLE(hazard_side_release_m)
        PARSE_FIELD_BOOL(hazard_speed_cap_enable)
        PARSE_FIELD_DOUBLE(hazard_speed_cap_floor)
        PARSE_FIELD_DOUBLE(hazard_prediction_horizon_s)
        PARSE_FIELD_DOUBLE(hazard_reverse_distance)
        PARSE_FIELD_DOUBLE(hazard_heading_threshold)
        PARSE_FIELD_DOUBLE(hazard_reverse_min_speed)
    )
    // clang-format on
};

std::shared_ptr<NavigationInterface> make_navigation(const NavigationConfiguration &config,
                                                     std::shared_ptr<ClockInterface> clock);
std::unique_ptr<NavigationConfiguration> parse_navigation_config(ConfigParser &parser);
std::unique_ptr<NavigationConfiguration> load_navigation_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections,
    const PlantConfiguration &plant);
}  // namespace auto_battlebot
