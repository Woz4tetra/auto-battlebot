#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "plant/config.hpp"
#include "robot_filter/motion_estimator_interface.hpp"

namespace auto_battlebot {

struct MotionEstimatorConfiguration {
    std::string type;
    virtual ~MotionEstimatorConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}

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
 * How opponent tracks are propagated. KALMAN runs the 6-state filter (constant velocity plus an
 * independent constant-yaw-rate heading) with innovation gating and coasts through dropout gaps.
 * HOLD pins each opponent at its last measured pose with no prediction of any kind, matching
 * DeadReckoningMotionEstimator's opponent behavior.
 */
enum class OpponentMode { KALMAN, HOLD };

struct KalmanMotionEstimatorConfiguration : public MotionEstimatorConfiguration {
    OurRobotMode our_robot_mode = OurRobotMode::DEAD_RECKONING;

    OpponentMode opponent_mode = OpponentMode::HOLD;

    /**
     * Continuous white-noise acceleration PSD (m^2/s^3) for the opponent constant-velocity
     * model. Placeholder until it is measured from fight replays; sized from ~10 m/s^2 peak
     * accelerations sustained over ~0.3 s maneuvers.
     */
    double opponent_accel_psd = 30.0;
    /** Continuous white-noise yaw acceleration PSD (rad^2/s^3) for the opponent heading block,
     * which runs a constant-yaw-rate model independent of the translation block. */
    double opponent_yaw_accel_psd = 100.0;
    /** Sigma (rad/s) on the opponent yaw rate when the heading block initializes. */
    double initial_yaw_rate_sigma = 20.0;
    /** Heading measurement sigma (radians) for opponent keypoint poses. Separate from
     * keypoint_heading_sigma_rad because that one also tunes the our-robot EKF. */
    double opponent_heading_sigma_rad = 0.15;
    /** Position measurement sigma (meters) for opponent keypoint poses, before the per-meter
     * range term. Separate from keypoint_position_sigma_m, which also tunes the our-robot EKF.
     * Blob centroids keep blob_position_sigma_m. */
    double opponent_position_sigma_m = 0.03;
    /** Consecutive opponent heading measurements past pi/2 of the track (flips) after which the
     * heading block re-initializes at the measured heading: a run of consistent flips means the
     * track had the front wrong, not the keypoint model. */
    int heading_flip_reinit_count = 3;
    /** Labels whose heading is never measured: full-body spinners and robots the keypoint model
     * cannot orient. Their heading sigma renders as infinite, which tells navigation the front is
     * unknown. */
    std::vector<Label> heading_blind_labels;

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

    double max_coast_s = 0.4;
    double opponent_max_coast_s = 0.1;

    double speed_measurement_min_dt_s = 0.08;
    /** Baseline age (seconds) past which the held reference pose is dropped rather than
     * differenced. A gap this long means dropped detections, and the robot's path between the
     * two poses is no longer a straight line worth differentiating. */
    double speed_measurement_max_dt_s = 0.25;
    double speed_position_sigma_m = 0.0035;
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
        PARSE_FIELD_DOUBLE(opponent_yaw_accel_psd)
        PARSE_FIELD_DOUBLE(initial_yaw_rate_sigma)
        PARSE_FIELD_DOUBLE(opponent_heading_sigma_rad)
        PARSE_FIELD_DOUBLE(opponent_position_sigma_m)
        heading_flip_reinit_count = static_cast<int>(
            parser.get_optional_int("heading_flip_reinit_count", heading_flip_reinit_count));
        for (const std::string &name :
             parser.get_optional_vector<std::string>("heading_blind_labels")) {
            const auto label = magic_enum::enum_cast<Label>(name);
            if (!label.has_value()) {
                throw std::invalid_argument("Invalid Label value: '" + name +
                                            "' for field 'heading_blind_labels'");
            }
            heading_blind_labels.push_back(*label);
        }
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
        PARSE_FIELD_DOUBLE(opponent_max_coast_s)
        PARSE_FIELD_DOUBLE(speed_measurement_min_dt_s)
        PARSE_FIELD_DOUBLE(speed_measurement_max_dt_s)
        PARSE_FIELD_DOUBLE(speed_position_sigma_m)
        PARSE_FIELD_DOUBLE(speed_sigma_floor_mps)
        PARSE_FIELD_DOUBLE(speed_gate_nis)
        parser.validate_no_extra_fields();
    }

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
