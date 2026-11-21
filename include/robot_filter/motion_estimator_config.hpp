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

struct KalmanMotionEstimatorConfiguration : public MotionEstimatorConfiguration {
    double opponent_accel_psd = 30.0;
    double opponent_yaw_accel_psd = 100.0;
    double initial_yaw_rate_sigma = 20.0;
    double opponent_heading_sigma_rad = 0.15;
    double opponent_position_sigma_m = 0.03;
    int heading_flip_reinit_count = 3;

    double keypoint_position_sigma_m = 0.03;
    double keypoint_heading_sigma_rad = 0.15;
    double position_sigma_per_meter = 0.0;

    double gate_nis = 11.83;
    int reinit_after_rejects = 5;
    double initial_velocity_sigma = 3.0;
    double covariance_floor = 1e-6;

    double clock_gap_warn_period_s = 5.0;

    double max_coast_s = 0.4;
    double max_position_sigma_m = 1.0;

    double speed_measurement_min_dt_s = 0.08;
    double speed_measurement_max_dt_s = 0.25;
    double speed_position_sigma_m = 0.0035;
    double speed_sigma_floor_mps = 0.25;
    double speed_gate_nis = 400.0;

    std::optional<JigPlantParams> plant;
    JigPlantNoiseParams plant_noise;

    KalmanMotionEstimatorConfiguration() { type = "KalmanMotionEstimator"; }

    void parse_fields(ConfigParser &parser) override {
        PARSE_FIELD_DOUBLE(opponent_accel_psd)
        PARSE_FIELD_DOUBLE(opponent_yaw_accel_psd)
        PARSE_FIELD_DOUBLE(initial_yaw_rate_sigma)
        PARSE_FIELD_DOUBLE(opponent_heading_sigma_rad)
        PARSE_FIELD_DOUBLE(opponent_position_sigma_m)
        heading_flip_reinit_count = static_cast<int>(
            parser.get_optional_int("heading_flip_reinit_count", heading_flip_reinit_count));
        PARSE_FIELD_DOUBLE(keypoint_position_sigma_m)
        PARSE_FIELD_DOUBLE(keypoint_heading_sigma_rad)
        PARSE_FIELD_DOUBLE(position_sigma_per_meter)
        PARSE_FIELD_DOUBLE(gate_nis)
        reinit_after_rejects =
            static_cast<int>(parser.get_optional_int("reinit_after_rejects", reinit_after_rejects));
        PARSE_FIELD_DOUBLE(initial_velocity_sigma)
        PARSE_FIELD_DOUBLE(covariance_floor)
        PARSE_FIELD_DOUBLE(clock_gap_warn_period_s)
        PARSE_FIELD_DOUBLE(max_coast_s)
        PARSE_FIELD_DOUBLE(max_position_sigma_m)
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
    }
};

std::unique_ptr<MotionEstimatorInterface> make_motion_estimator(
    const MotionEstimatorConfiguration &config);

}  // namespace auto_battlebot
