#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "enums/opponent_mode.hpp"
#include "enums/our_robot_mode.hpp"
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
    std::optional<JigPlantParams> plant;

    DeadReckoningMotionEstimatorConfiguration() { type = "DeadReckoningMotionEstimator"; }

    PARSE_CONFIG_FIELDS()

    void apply_plant(const PlantConfiguration &plant_config) override {
        if (!plant_config.params.has_value()) {
            throw ConfigValidationError(
                "type = 'DeadReckoningMotionEstimator' requires the top-level [plant] table; see "
                "docs/reference/control/plant_backed_control.md");
        }
        plant = *plant_config.params;
    }
};

struct KalmanMotionEstimatorConfiguration : public MotionEstimatorConfiguration {
    OurRobotMode our_robot_mode = OurRobotMode::DEAD_RECKONING;

    OpponentMode opponent_mode = OpponentMode::HOLD;

    double opponent_accel_psd = 30.0;
    double opponent_yaw_accel_psd = 100.0;
    double initial_yaw_rate_sigma = 20.0;
    double opponent_heading_sigma_rad = 0.15;
    double opponent_position_sigma_m = 0.03;
    int heading_flip_reinit_count = 3;
    std::vector<Label> heading_blind_labels;

    double keypoint_position_sigma_m = 0.03;
    double blob_position_sigma_m = 0.10;
    double keypoint_heading_sigma_rad = 0.15;
    double position_sigma_per_meter = 0.0;

    double gate_nis = 11.83;
    int reinit_after_rejects = 5;
    double initial_velocity_sigma = 3.0;
    double covariance_floor = 1e-6;

    double coast_stale_age_s = 0.1;
    double clock_gap_warn_period_s = 5.0;

    double max_coast_s = 0.4;
    double opponent_max_coast_s = 0.1;

    double speed_measurement_min_dt_s = 0.08;
    double speed_measurement_max_dt_s = 0.25;
    double speed_position_sigma_m = 0.0035;
    double speed_sigma_floor_mps = 0.25;
    double speed_gate_nis = 400.0;

    std::optional<JigPlantParams> plant;
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
        PARSE_FIELD_DOUBLE(coast_stale_age_s)
        PARSE_FIELD_DOUBLE(clock_gap_warn_period_s)
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
                "type = 'KalmanMotionEstimator' requires the top-level [plant] table; see "
                "docs/reference/control/plant_backed_control.md");
        }
    }
};

std::unique_ptr<MotionEstimatorInterface> make_motion_estimator(
    const MotionEstimatorConfiguration &config);

}  // namespace auto_battlebot
