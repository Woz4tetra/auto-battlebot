#pragma once

#include <cmath>
#include <map>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "robot_filter/keypoint_heights.hpp"
#include "robot_filter/motion_estimator_config.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct RobotFilterConfiguration {
    std::string type;
    virtual ~RobotFilterConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}

    virtual void apply_plant([[maybe_unused]] const PlantConfiguration &plant) {}
};

struct NoopRobotFilterConfiguration : public RobotFilterConfiguration {
    NoopRobotFilterConfiguration() { type = "NoopRobotFilter"; }

    PARSE_CONFIG_FIELDS()
};

struct GroundTruthRobotFilterConfiguration : public RobotFilterConfiguration {
    int neutral_count = 0;

    GroundTruthRobotFilterConfiguration() { type = "GroundTruthRobotFilter"; }

    PARSE_CONFIG_FIELDS(PARSE_FIELD(neutral_count))
};

struct RobotFrontBackFilterConfiguration : public RobotFilterConfiguration {
    std::vector<KeypointLabel> front_keypoints;
    std::vector<KeypointLabel> back_keypoints;
    std::map<Label, std::vector<FrameId>> label_to_frame_ids;
    FrameId default_frame_id;
    double max_jump_distance = 0.3;
    int max_consecutive_jump_rejects = 5;
    double field_bounds_margin_meters = 0.2;
    KeypointHeights keypoint_heights;
    std::shared_ptr<MotionEstimatorConfiguration> motion_estimator =
        std::make_shared<KalmanMotionEstimatorConfiguration>();

    RobotFrontBackFilterConfiguration() { type = "RobotFrontBackFilter"; }

    void parse_fields(ConfigParser &parser) override {
        front_keypoints = parse_keypoints(parser, "front_keypoints");
        back_keypoints = parse_keypoints(parser, "back_keypoints");
        parse_label_to_frame_id(parser, "label_mapping");
        PARSE_ENUM_REQUIRED(default_frame_id, FrameId);
        PARSE_FIELD_DOUBLE(max_jump_distance)
        max_consecutive_jump_rejects = static_cast<int>(
            parser.get_optional_int("max_consecutive_jump_rejects", max_consecutive_jump_rejects));
        field_bounds_margin_meters =
            parser.get_optional_double("field_bounds_margin_meters", field_bounds_margin_meters);
        keypoint_heights.default_meters =
            parser.get_optional_double("keypoint_height_meters", keypoint_heights.default_meters);
        parse_keypoint_heights_per_label(parser, "keypoint_height_meters_per_label");
        parse_robot_sizes_per_label(parser, "robot_size_meters_per_label");
        parse_footprint_center_offsets(parser, "footprint_center_offset_per_label");
        parse_motion_estimator(parser);
        parser.validate_no_extra_fields();
    }

    void apply_plant(const PlantConfiguration &plant) override {
        motion_estimator->apply_plant(plant);
    }

    void parse_motion_estimator(ConfigParser &parser) {
        const toml::table *table_ptr = parser.get_table("motion_estimator");
        if (!table_ptr) {
            return;
        }
        ConfigParser sub_parser(*table_ptr, "robot_filter.motion_estimator");
        motion_estimator =
            ConfigFactory<MotionEstimatorConfiguration>::instance().create_and_parse(sub_parser);
    }

    std::map<Label, Size> robot_size_meters_per_label;

    void parse_robot_sizes_per_label(ConfigParser &parser, const std::string &field_name) {
        const toml::table *table_ptr = parser.get_table(field_name);
        if (!table_ptr) {
            return;
        }
        for (const auto &[key, value] : *table_ptr) {
            std::string label_str(key.str());
            auto label_opt = magic_enum::enum_cast<Label>(label_str);
            if (!label_opt.has_value()) {
                throw ConfigValidationError("Invalid Label: " + label_str);
            }
            const toml::array *dims = value.as_array();
            if (dims == nullptr || dims->size() != 3) {
                throw ConfigValidationError("robot_size_meters_per_label['" + label_str +
                                            "'] must be [x, y, z]");
            }
            Size size;
            double *fields[3] = {&size.x, &size.y, &size.z};
            for (size_t i = 0; i < 3; ++i) {
                auto component = (*dims)[i].value<double>();
                if (!component.has_value() || *component <= 0.0) {
                    throw ConfigValidationError("robot_size_meters_per_label['" + label_str +
                                                "'] components must be numbers > 0");
                }
                *fields[i] = *component;
            }
            robot_size_meters_per_label[label_opt.value()] = size;
        }
    }

    std::map<Label, Position2D> footprint_center_offset_per_label;

    void parse_footprint_center_offsets(ConfigParser &parser, const std::string &field_name) {
        const toml::table *table_ptr = parser.get_table(field_name);
        if (!table_ptr) {
            return;
        }
        for (const auto &[key, value] : *table_ptr) {
            const std::string label_str(key.str());
            auto label_opt = magic_enum::enum_cast<Label>(label_str);
            if (!label_opt.has_value()) {
                throw ConfigValidationError("Invalid Label: " + label_str);
            }
            const std::string error = field_name + "['" + label_str + "'] must be [x, y] in metres";
            const toml::array *xy = value.as_array();
            if (xy == nullptr || xy->size() != 2) throw ConfigValidationError(error);
            Position2D offset;
            double *fields[2] = {&offset.x, &offset.y};
            for (size_t i = 0; i < 2; ++i) {
                const auto component = (*xy)[i].value<double>();
                if (!component.has_value() || !std::isfinite(*component)) {
                    throw ConfigValidationError(error);
                }
                *fields[i] = *component;
            }
            footprint_center_offset_per_label[label_opt.value()] = offset;
        }
    }

    void parse_keypoint_heights_per_label(ConfigParser &parser, const std::string &field_name) {
        const toml::table *table_ptr = parser.get_table(field_name);
        if (!table_ptr) {
            return;
        }

        for (const auto &[key, value] : *table_ptr) {
            std::string label_str(key.str());
            auto label_opt = magic_enum::enum_cast<Label>(label_str);
            if (!label_opt.has_value()) {
                throw ConfigValidationError("Invalid Label: " + label_str);
            }
            auto height = value.template value<double>();
            if (!height.has_value()) {
                throw ConfigValidationError("Value for '" + label_str + "' must be a number");
            }
            keypoint_heights.per_label_meters[label_opt.value()] = *height;
        }
    }

    void parse_label_to_frame_id(ConfigParser &parser, const std::string &field_name) {
        const toml::table *table_ptr = parser.get_table(field_name);
        if (!table_ptr) {
            throw ConfigValidationError("Missing required field '" + field_name + "'");
        }

        const toml::table &table = *table_ptr;

        for (const auto &[key, value] : table) {
            std::string label_str(key.str());

            auto label_opt = magic_enum::enum_cast<Label>(label_str);
            if (!label_opt.has_value()) {
                throw ConfigValidationError("Invalid Label: " + label_str);
            }
            Label label = label_opt.value();

            auto frame_id_array = value.as_array();
            if (!frame_id_array) {
                throw ConfigValidationError("Value for '" + label_str + "' must be an array");
            }

            std::vector<FrameId> frame_ids;
            for (const auto &fr_toml : *frame_id_array) {
                auto fr_str = fr_toml.template value<std::string>();
                if (fr_str) {
                    auto fr_opt = magic_enum::enum_cast<FrameId>(*fr_str);
                    if (fr_opt.has_value()) {
                        frame_ids.push_back(fr_opt.value());
                    } else {
                        throw ConfigValidationError("Invalid KeypointLabel: " + *fr_str);
                    }
                }
            }

            label_to_frame_ids[label] = frame_ids;
        }
    }

    std::vector<KeypointLabel> parse_keypoints(ConfigParser &parser,
                                               const std::string &field_name) {
        const toml::array *array_ptr = parser.get_array(field_name);
        if (!array_ptr) {
            throw ConfigValidationError("Missing required field '" + field_name + "'");
        }

        if (!array_ptr) {
            throw ConfigValidationError("Value for front_keypoints must be an array");
        }
        auto keypoint_array_table = *array_ptr;
        std::vector<KeypointLabel> keypoint_labels;
        for (const auto &kp_toml : keypoint_array_table) {
            auto kp_str = kp_toml.template value<std::string>();
            if (kp_str) {
                auto kp_opt = magic_enum::enum_cast<KeypointLabel>(*kp_str);
                if (kp_opt.has_value()) {
                    keypoint_labels.push_back(kp_opt.value());
                } else {
                    throw ConfigValidationError("Invalid KeypointLabel: " + *kp_str);
                }
            }
        }
        return keypoint_labels;
    }
};

std::shared_ptr<RobotFilterInterface> make_robot_filter(const RobotFilterConfiguration &config,
                                                        std::shared_ptr<ClockInterface> clock);
std::unique_ptr<RobotFilterConfiguration> parse_robot_filter_config(ConfigParser &parser);
std::unique_ptr<RobotFilterConfiguration> load_robot_filter_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections,
    const PlantConfiguration &plant);
}  // namespace auto_battlebot
