#pragma once

#include <memory>
#include <string>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "robot_filter/motion_estimator_interface.hpp"

namespace auto_battlebot {

struct MotionEstimatorConfiguration {
    std::string type;
    virtual ~MotionEstimatorConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct DeadReckoningMotionEstimatorConfiguration : public MotionEstimatorConfiguration {
    DeadReckoningMotionEstimatorConfiguration() { type = "DeadReckoningMotionEstimator"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields -- behavior lives in the outer robot_filter section
    )
};

struct KalmanMotionEstimatorConfiguration : public MotionEstimatorConfiguration {
    /**
     * How the our-robot track is propagated. "dead_reckoning" applies the last commanded
     * velocity directly, matching DeadReckoningMotionEstimator for that track. "ekf" runs the
     * 5-state EKF and requires a registered plant model; none exists until a model ladder rung
     * is selected, so "ekf" is rejected at config validation with a message saying why.
     */
    std::string our_robot_mode = "dead_reckoning";

    /**
     * Continuous white-noise acceleration PSD (m^2/s^3) for the opponent constant-velocity
     * model. Placeholder until it is measured from fight replays; sized from ~10 m/s^2 peak
     * accelerations sustained over ~0.3 s maneuvers.
     */
    double opponent_accel_psd = 30.0;

    /** Position measurement sigma (meters) for keypoint-derived poses. Placeholder until the
     * joint jig and camera session measures R. */
    double keypoint_position_sigma_m = 0.03;
    /** Position measurement sigma (meters) for blob centroids. */
    double blob_position_sigma_m = 0.10;
    /** Extra position sigma per meter of camera range, applied on top of the base sigma. Zero
     * disables range dependence. */
    double position_sigma_per_meter = 0.0;

    /** Innovation gate on normalized innovation squared, chi-square with 2 DOF. 11.83 is the
     * 99.7% quantile. Values <= 0 disable gating. */
    double gate_nis = 11.83;
    /** Consecutive gated-out measurements on one track before the track resets to the next
     * accepted measurement. */
    int reinit_after_rejects = 5;
    /** Sigma (m/s) on each velocity axis when a track initializes from its first measurement. */
    double initial_velocity_sigma = 3.0;
    /** Floor on the diagonal of P so the covariance cannot collapse and lock out measurements. */
    double covariance_floor = 1e-6;

    /**
     * Coast horizon (seconds). Unmeasured tracks predict forward with growing covariance up to
     * this age, then hold position. Default sits at the measured p90-p95 of perception dropout
     * gaps; predicting multi-second gaps with a short-horizon model is fiction.
     */
    double max_coast_s = 0.4;

    /** Body speed (m/s) below which an opponent's velocity-derived heading is not trusted and
     * the last measured heading is kept. */
    double min_heading_speed = 0.3;

    KalmanMotionEstimatorConfiguration() { type = "KalmanMotionEstimator"; }

    void parse_fields(ConfigParser &parser) override {
        our_robot_mode = parser.get_optional_string("our_robot_mode", our_robot_mode);
        if (our_robot_mode != "dead_reckoning" && our_robot_mode != "ekf") {
            throw ConfigValidationError("Invalid our_robot_mode '" + our_robot_mode +
                                        "': expected 'dead_reckoning' or 'ekf'");
        }
        if (our_robot_mode == "ekf") {
            throw ConfigValidationError(
                "our_robot_mode = 'ekf' requires a plant model, and none is registered: the "
                "velocity jig plant fit has not passed its acceptance criteria and no model "
                "ladder rung is selected. Use 'dead_reckoning' until then. See "
                "docs/experiments/kalman_filter/cpp_implementation_plan.md");
        }
        PARSE_FIELD_DOUBLE(opponent_accel_psd)
        PARSE_FIELD_DOUBLE(keypoint_position_sigma_m)
        PARSE_FIELD_DOUBLE(blob_position_sigma_m)
        PARSE_FIELD_DOUBLE(position_sigma_per_meter)
        PARSE_FIELD_DOUBLE(gate_nis)
        reinit_after_rejects =
            static_cast<int>(parser.get_optional_int("reinit_after_rejects", reinit_after_rejects));
        PARSE_FIELD_DOUBLE(initial_velocity_sigma)
        PARSE_FIELD_DOUBLE(covariance_floor)
        PARSE_FIELD_DOUBLE(max_coast_s)
        PARSE_FIELD_DOUBLE(min_heading_speed)
        parser.validate_no_extra_fields();
    }
};

std::unique_ptr<MotionEstimatorInterface> make_motion_estimator(
    const MotionEstimatorConfiguration &config);

}  // namespace auto_battlebot
