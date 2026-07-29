#pragma once

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "robot_filter/robot_filter_interface.hpp"
#include "robot_filter/robot_keypoint_tracker.hpp"
#include "time/clock_interface.hpp"

namespace auto_battlebot {
struct RobotFilterConfiguration {
    std::string type;
    virtual ~RobotFilterConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopRobotFilterConfiguration : public RobotFilterConfiguration {
    NoopRobotFilterConfiguration() { type = "NoopRobotFilter"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct GroundTruthRobotFilterConfiguration : public RobotFilterConfiguration {
    GroundTruthRobotFilterConfiguration() { type = "GroundTruthRobotFilter"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields -- reads GT poses from SimConnection
    )
};

struct RobotFrontBackSimpleFilterConfiguration : public RobotFilterConfiguration {
    std::vector<KeypointLabel> front_keypoints;
    std::vector<KeypointLabel> back_keypoints;
    std::map<Label, std::vector<FrameId>> label_to_frame_ids;
    FrameId default_frame_id;
    double max_jump_distance = 0.3;
    int max_consecutive_jump_rejects = 5;
    double blob_overwrite_min_distance_meters = 0.20;
    double blob_overwrite_size_scale = 0.5;
    double field_bounds_margin_meters = 0.2;
    /**
     * How long (seconds) an unmeasured our-robot track is held before it is dropped from the
     * output. After this window without a keypoint confirmation the stale "ours" detection is
     * decayed away instead of being predicted forward indefinitely. A value <= 0 disables the
     * decay (legacy behavior: hold forever).
     */
    double our_robot_hold_window_s = 0.15;
    /**
     * Radius (meters) around our robot's last held pose in which blob detections are discarded
     * while the keypoint model is not detecting our robot. The blob model cannot identify our
     * robot, so without this an undetected own robot is emitted as an opponent at our own
     * position. A value <= 0 disables the suppression.
     */
    double our_keypoint_dropout_blob_radius_meters = 0.4;
    /**
     * How long (seconds) after our robot's last keypoint-confirmed frame the suppression radius
     * stays active. Outlives our_robot_hold_window_s on purpose: the track decays sooner than a
     * typical keypoint dropout ends. A value <= 0 disables the suppression.
     */
    double our_keypoint_dropout_blob_window_s = 1.0;
    RobotKeypointTrackerConfig robot_keypoint_tracker_config;

    RobotFrontBackSimpleFilterConfiguration() { type = "RobotFrontBackSimpleFilter"; }

    void parse_fields(ConfigParser &parser) override {
        front_keypoints = parse_keypoints(parser, "front_keypoints");
        back_keypoints = parse_keypoints(parser, "back_keypoints");
        parse_label_to_frame_id(parser, "label_mapping");
        PARSE_ENUM_REQUIRED(default_frame_id, FrameId);
        PARSE_FIELD_DOUBLE(max_jump_distance)
        max_consecutive_jump_rejects = static_cast<int>(
            parser.get_optional_int("max_consecutive_jump_rejects", max_consecutive_jump_rejects));
        blob_overwrite_min_distance_meters = parser.get_optional_double(
            "blob_overwrite_min_distance_meters", blob_overwrite_min_distance_meters);
        blob_overwrite_size_scale =
            parser.get_optional_double("blob_overwrite_size_scale", blob_overwrite_size_scale);
        field_bounds_margin_meters =
            parser.get_optional_double("field_bounds_margin_meters", field_bounds_margin_meters);
        our_robot_hold_window_s =
            parser.get_optional_double("our_robot_hold_window_s", our_robot_hold_window_s);
        our_keypoint_dropout_blob_radius_meters = parser.get_optional_double(
            "our_keypoint_dropout_blob_radius_meters", our_keypoint_dropout_blob_radius_meters);
        our_keypoint_dropout_blob_window_s = parser.get_optional_double(
            "our_keypoint_dropout_blob_window_s", our_keypoint_dropout_blob_window_s);
        parse_robot_keypoint_tracker_config(parser);
        parser.validate_no_extra_fields();
    }

    void parse_robot_keypoint_tracker_config(ConfigParser &parser) {
        robot_keypoint_tracker_config.min_length_meters = parser.get_optional_double(
            "robot_blob_min_length_meters", robot_keypoint_tracker_config.min_length_meters);
        robot_keypoint_tracker_config.max_length_meters = parser.get_optional_double(
            "robot_blob_max_length_meters", robot_keypoint_tracker_config.max_length_meters);
        robot_keypoint_tracker_config.min_confidence = parser.get_optional_double(
            "robot_blob_min_confidence", robot_keypoint_tracker_config.min_confidence);
        robot_keypoint_tracker_config.keypoint_heights.default_meters = parser.get_optional_double(
            "keypoint_height_meters",
            robot_keypoint_tracker_config.keypoint_heights.default_meters);
        parse_keypoint_heights_per_label(parser, "keypoint_height_meters_per_label");
        robot_keypoint_tracker_config.max_candidates = static_cast<int>(parser.get_optional_int(
            "robot_blob_max_candidates", robot_keypoint_tracker_config.max_candidates));
    }

    void parse_keypoint_heights_per_label(ConfigParser &parser, const std::string &field_name) {
        const toml::table *table_ptr = parser.get_table(field_name);
        if (!table_ptr) {
            return;  // Optional; labels fall back to keypoint_height_meters.
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
            robot_keypoint_tracker_config.keypoint_heights.per_label_meters[label_opt.value()] =
                *height;
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

            // Parse the label
            auto label_opt = magic_enum::enum_cast<Label>(label_str);
            if (!label_opt.has_value()) {
                throw ConfigValidationError("Invalid Label: " + label_str);
            }
            Label label = label_opt.value();

            // Parse the array of frame IDs
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
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
