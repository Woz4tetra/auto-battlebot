#pragma once

#include <memory>
#include <optional>
#include <string>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "plant/config.hpp"
#include "robot_filter/motion_estimator_interface.hpp"

namespace auto_battlebot {

struct MotionEstimatorConfiguration {
    std::string type;
    virtual ~MotionEstimatorConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}

    /**
     * Hand over the shared [plant] table, after parse_fields. Separate from parsing because the
     * plant is a top-level section: this config's own parser never sees it. Estimators that
     * ignore the plant keep the no-op.
     */
    virtual void apply_plant([[maybe_unused]] const PlantConfiguration &plant) {}
};

struct DeadReckoningMotionEstimatorConfiguration : public MotionEstimatorConfiguration {
    /**
     * Fitted gains from the shared top-level [plant] table, stamped in by apply_plant. The
     * estimator scales the normalized stick command to body velocity with k_fwd/k_rev/k_ang and
     * has no other source for what full stick means. Optional only so apply_plant can raise a
     * config error instead of the constructor asserting.
     */
    std::optional<JigPlantParams> plant;

    DeadReckoningMotionEstimatorConfiguration() { type = "DeadReckoningMotionEstimator"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields -- behavior lives in the outer robot_filter section
    )

    void apply_plant(const PlantConfiguration &plant_config) override {
        if (!plant_config.params.has_value()) {
            throw ConfigValidationError(
                "type = 'DeadReckoningMotionEstimator' requires the top-level [plant] table: it "
                "scales the normalized stick command to body velocity with the fitted "
                "k_fwd/k_rev/k_ang and has no defaults for them. A zero plant would freeze every "
                "prediction without saying so. See docs/plant_backed_control.md");
        }
        plant = *plant_config.params;
    }
};

/**
 * How the our-robot track is propagated. DEAD_RECKONING applies the last commanded velocity
 * directly, matching DeadReckoningMotionEstimator for that track. EKF runs the 5-state EKF
 * with JigPlantModel built from the plant table, and is rejected when that table is absent.
 */
enum class OurRobotMode { DEAD_RECKONING, EKF };

/**
 * How opponent tracks are propagated. KALMAN runs the constant-velocity filter with
 * innovation gating and coasts through dropout gaps. HOLD pins each opponent at its last
 * measured pose with no prediction of any kind, matching DeadReckoningMotionEstimator's
 * opponent behavior.
 */
enum class OpponentMode { KALMAN, HOLD };

struct KalmanMotionEstimatorConfiguration : public MotionEstimatorConfiguration {
    /** The plant fit has not passed acceptance (heading misses by 4x at the 400 ms coast
     * horizon), so production configs stay on DEAD_RECKONING; sim and playback experiments
     * opt in through the extends chain. */
    OurRobotMode our_robot_mode = OurRobotMode::DEAD_RECKONING;

    /** Opponent prediction is disabled for now; flip to KALMAN to re-enable. */
    OpponentMode opponent_mode = OpponentMode::HOLD;

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
    /** Heading measurement sigma (radians) for keypoint-derived poses, our-robot EKF only.
     * Placeholder until the joint jig and camera session measures R. */
    double keypoint_heading_sigma_rad = 0.15;
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

    /**
     * Baseline (seconds) for the our-robot forward-speed observation, differenced between two
     * accepted keypoint poses. Nothing else in the update observes v directly, so without this
     * the speed the motion profile brakes on is whatever the plant model predicted. One frame
     * of baseline is too short to be useful: at 30 Hz and a 0.03 m position sigma the
     * difference carries ~1.3 m/s of noise, so the reference pose is held until the gap reaches
     * this value (about three frames) and the sigma falls with it.
     */
    double speed_measurement_min_dt_s = 0.08;
    /** Baseline age (seconds) past which the held reference pose is dropped rather than
     * differenced. A gap this long means dropped detections, and the robot's path between the
     * two poses is no longer a straight line worth differentiating. */
    double speed_measurement_max_dt_s = 0.25;
    /** Position noise (meters) the forward-speed sigma is derived from. Deliberately not
     * keypoint_position_sigma_m: that one is a conservative placeholder that also sets the
     * position gate, while differencing needs the real frame-to-frame scatter. Measured at
     * 2.7 mm median residual about a 9-frame smooth on the 2026-08-23 20-37 recording, taken
     * here at 3.5 mm to leave room for the smoother following some of the noise. */
    double speed_position_sigma_m = 0.0035;
    /** Floor on the derived forward-speed sigma (m/s). Position scatter is not the whole error:
     * the chord understates an arc, the projection inherits the heading error, and the two
     * endpoint stamps are not exact. Set from the residual against a 150 ms chord reference on
     * the same recording, 0.168 m/s median absolute, which is 0.25 sigma if Gaussian. */
    double speed_sigma_floor_mps = 0.25;
    /** Chi-square gate for the forward-speed update alone, deliberately looser than gate_nis.
     * That gate protects a pose the track cannot recover without; this one guards a state whose
     * fallback is the plant model's own guess, which the innovations say is worse than a noisy
     * measurement. Sized to pass the innovations the current process noise makes look
     * impossible. */
    double speed_gate_nis = 400.0;

    /**
     * Jig-fitted plant parameters, from the shared top-level [plant] table, stamped in by
     * apply_plant. Absent unless the config carries the table. Presence does not enable
     * our_robot_mode = EKF: that stays rejected until the plant fit passes acceptance. Sim,
     * playback, and tests construct JigPlantModel from here, which is what keeps the parameters
     * in the config extends chain instead of a separate runtime file.
     */
    std::optional<JigPlantParams> plant;
    /** Process noise PSDs for the plant model, from [plant.process_noise]. Fields are optional;
     * defaults are the fit_process_noise.py output baked into JigPlantNoiseParams. */
    JigPlantNoiseParams plant_noise;

    KalmanMotionEstimatorConfiguration() { type = "KalmanMotionEstimator"; }

    void parse_fields(ConfigParser &parser) override {
        PARSE_ENUM(our_robot_mode, OurRobotMode)
        PARSE_ENUM(opponent_mode, OpponentMode)
        PARSE_FIELD_DOUBLE(opponent_accel_psd)
        PARSE_FIELD_DOUBLE(keypoint_position_sigma_m)
        PARSE_FIELD_DOUBLE(blob_position_sigma_m)
        PARSE_FIELD_DOUBLE(keypoint_heading_sigma_rad)
        PARSE_FIELD_DOUBLE(position_sigma_per_meter)
        PARSE_FIELD_DOUBLE(gate_nis)
        reinit_after_rejects =
            static_cast<int>(parser.get_optional_int("reinit_after_rejects", reinit_after_rejects));
        PARSE_FIELD_DOUBLE(initial_velocity_sigma)
        PARSE_FIELD_DOUBLE(covariance_floor)
        PARSE_FIELD_DOUBLE(max_coast_s)
        PARSE_FIELD_DOUBLE(min_heading_speed)
        PARSE_FIELD_DOUBLE(speed_measurement_min_dt_s)
        PARSE_FIELD_DOUBLE(speed_measurement_max_dt_s)
        PARSE_FIELD_DOUBLE(speed_position_sigma_m)
        PARSE_FIELD_DOUBLE(speed_sigma_floor_mps)
        PARSE_FIELD_DOUBLE(speed_gate_nis)
        parser.validate_no_extra_fields();
    }

    /**
     * Both our-robot modes need the fit: EKF propagates through JigPlantModel, DEAD_RECKONING
     * scales stick to body velocity with k_fwd/k_rev/k_ang. The check lives here rather than in
     * parse_fields because the plant is a top-level section this config's parser cannot see.
     */
    void apply_plant(const PlantConfiguration &plant_config) override {
        plant = plant_config.params;
        plant_noise = plant_config.noise;
        if (!plant.has_value()) {
            throw ConfigValidationError(
                "type = 'KalmanMotionEstimator' requires the top-level [plant] table: the EKF "
                "our-robot arm propagates through JigPlantModel and the dead-reckoning arm scales "
                "stick to body velocity with the fitted gains. Neither has defaults. See "
                "docs/experiments/kalman_filter/plant_model_poc_plan.md");
        }
    }
};

std::unique_ptr<MotionEstimatorInterface> make_motion_estimator(
    const MotionEstimatorConfiguration &config);

}  // namespace auto_battlebot
