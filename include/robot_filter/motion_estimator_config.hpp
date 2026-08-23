#pragma once

#include <memory>
#include <optional>
#include <string>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "robot_filter/jig_plant_model.hpp"
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

    /**
     * Jig-fitted plant parameters, from the [robot_filter.motion_estimator.plant] table.
     * Absent unless the config carries the table. Presence does not enable our_robot_mode =
     * "ekf": that stays rejected until the plant fit passes acceptance. Sim, playback, and
     * tests construct JigPlantModel from here, which is what keeps the parameters in the
     * config extends chain instead of a separate runtime file.
     */
    std::optional<JigPlantParams> plant;
    /** Process noise PSDs for the plant model, [robot_filter.motion_estimator.plant.process_noise].
     * Fields are optional; defaults are the fit_process_noise.py output baked into
     * JigPlantNoiseParams. */
    JigPlantNoiseParams plant_noise;

    KalmanMotionEstimatorConfiguration() { type = "KalmanMotionEstimator"; }

    void parse_fields(ConfigParser &parser) override {
        our_robot_mode = parser.get_optional_string("our_robot_mode", our_robot_mode);
        if (our_robot_mode != "dead_reckoning" && our_robot_mode != "ekf") {
            throw ConfigValidationError("Invalid our_robot_mode '" + our_robot_mode +
                                        "': expected 'dead_reckoning' or 'ekf'");
        }
        if (our_robot_mode == "ekf") {
            throw ConfigValidationError(
                "our_robot_mode = 'ekf' is not available: the velocity jig plant fit has not "
                "passed its acceptance criteria (heading misses by 4x at the 400 ms coast "
                "horizon), so the EKF our-robot arm is not wired to the plant model. Use "
                "'dead_reckoning' until then. See "
                "docs/experiments/kalman_filter/plant_model_poc_plan.md");
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
        parse_plant(parser);
        parser.validate_no_extra_fields();
    }

    /**
     * Every plant field is required: these are fitted physical parameters, and a partly
     * defaulted plant would predict with a mix of fit and zero that no experiment produced.
     * A ladder rung disables a term by writing it as zero in the config, same as the fit
     * output files.
     */
    void parse_plant(ConfigParser &parser) {
        const toml::table *table_ptr = parser.get_table("plant");
        if (!table_ptr) {
            return;  // Optional; no plant model is constructible without it.
        }
        ConfigParser sub_parser(*table_ptr, "robot_filter.motion_estimator.plant");
        JigPlantParams params;
        params.dz_lin_fwd = sub_parser.get_required_double("dz_lin_fwd");
        params.dz_lin_rev = sub_parser.get_required_double("dz_lin_rev");
        params.dz_ang_l = sub_parser.get_required_double("dz_ang_l");
        params.dz_ang_r = sub_parser.get_required_double("dz_ang_r");
        params.k_fwd = sub_parser.get_required_double("k_fwd");
        params.k_rev = sub_parser.get_required_double("k_rev");
        params.k_ang = sub_parser.get_required_double("k_ang");
        params.tau_lin_a = sub_parser.get_required_double("tau_lin_a");
        params.tau_lin_d = sub_parser.get_required_double("tau_lin_d");
        params.tau_ang_a = sub_parser.get_required_double("tau_ang_a");
        params.tau_ang_d = sub_parser.get_required_double("tau_ang_d");
        params.delay_s = sub_parser.get_required_double("delay_s");
        params.c_sb = sub_parser.get_required_double("c_sb");
        params.c_ad = sub_parser.get_required_double("c_ad");
        params.c_drift = sub_parser.get_required_double("c_drift");
        params.c_drift_bias = sub_parser.get_required_double("c_drift_bias");
        parse_plant_noise(sub_parser);
        sub_parser.validate_no_extra_fields();
        plant = params;
    }

    void parse_plant_noise(ConfigParser &plant_parser) {
        const toml::table *table_ptr = plant_parser.get_table("process_noise");
        if (!table_ptr) {
            return;  // Optional; the defaults are the baked-in fit values.
        }
        ConfigParser sub_parser(*table_ptr, "robot_filter.motion_estimator.plant.process_noise");
        plant_noise.q_along = sub_parser.get_optional_double("q_along", plant_noise.q_along);
        plant_noise.q_cross = sub_parser.get_optional_double("q_cross", plant_noise.q_cross);
        plant_noise.q_heading = sub_parser.get_optional_double("q_heading", plant_noise.q_heading);
        plant_noise.scale_factor =
            sub_parser.get_optional_double("scale_factor", plant_noise.scale_factor);
        plant_noise.heading_scale_factor = sub_parser.get_optional_double(
            "heading_scale_factor", plant_noise.heading_scale_factor);
        plant_noise.heading_random_walk =
            sub_parser.get_optional_double("heading_random_walk", plant_noise.heading_random_walk);
        plant_noise.delay_jitter_s =
            sub_parser.get_optional_double("delay_jitter_s", plant_noise.delay_jitter_s);
        sub_parser.validate_no_extra_fields();
    }
};

std::unique_ptr<MotionEstimatorInterface> make_motion_estimator(
    const MotionEstimatorConfiguration &config);

}  // namespace auto_battlebot
