#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "config/enum_map_config.hpp"
#include "data_structures.hpp"
#include "engine_selector/config.hpp"
#include "enums/keypoint_label.hpp"
#include "enums/label.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {
struct KeypointModelConfiguration {
    std::string type;
    virtual ~KeypointModelConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopKeypointModelConfiguration : public KeypointModelConfiguration {
    NoopKeypointModelConfiguration() { type = "NoopKeypointModel"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct YoloKeypointModelConfiguration : public KeypointModelConfiguration {
    EngineSelectorConfiguration engine;
    float threshold = 0.50f;
    float iou_threshold = 0.45f;
    float letterbox_padding = 0.1f;
    int image_size = 640;
    LabelToKeypointMapConfiguration label_map;
    bool debug_visualization = false;
    std::vector<Label> label_indices;

    YoloKeypointModelConfiguration() { type = "YoloKeypointModel"; }

    void parse_fields(ConfigParser &parser) override {
        engine.parse(parser, "engine");
        threshold = parser.get_optional_double("threshold", threshold);
        iou_threshold = parser.get_optional_double("iou_threshold", iou_threshold);
        letterbox_padding = parser.get_optional_double("letterbox_padding", letterbox_padding);
        image_size = parser.get_optional_int("image_size", image_size);
        label_map.parse(parser, "label_map");
        debug_visualization = parser.get_optional_bool("debug_visualization", debug_visualization);
        std::vector<std::string> label_indices_str =
            parser.get_optional_vector<std::string>("label_indices");
        for (std::string label_str : label_indices_str) {
            auto enum_val = magic_enum::enum_cast<Label>(label_str);
            if (!enum_val.has_value()) {
                throw std::invalid_argument("Invalid value: '" + label_str +
                                            "' for field 'label_indices'");
            }
            label_indices.push_back(enum_val.value());
        }

        parser.validate_no_extra_fields();
    }
};

/** One robot tag's mounting: `rotation` (row-major R_body_tag) and `translation_m` map points in
 *  the OpenCV marker frame (origin at the tag centre, x right and y up in the printed image, z out
 *  of the printed face) into the robot body frame (FLU at the axle midpoint). */
struct AprilTagMountConfiguration {
    int id = 0;
    std::array<double, 3> translation_m{};
    std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

struct AprilTagKeypointModelConfiguration : public KeypointModelConfiguration {
    /** Edge of the black square, metres (the object points solvePnP sees). */
    double tag_size_m = 0.064;
    /** The robot's tags. Which one faces up comes from its mount rotation, not its position. */
    std::vector<int> robot_tag_ids = {41, 76};
    /** Search window around the last detection, grown by this on every side. */
    int roi_margin_px = 80;
    /** A tag pose worse than this emits no keypoints (it is still recorded). */
    double max_reprojection_error_px = 2.0;
    /** Body-frame points projected as the front and back keypoints. */
    std::array<double, 3> front_keypoint_m{0.06, 0.0, 0.0};
    std::array<double, 3> back_keypoint_m{-0.06, 0.0, 0.0};
    /** Axle above the floor, the wheel radius. Keypoint heights are measured from here. */
    double axle_height_m = 0.025;
    /** Nose-down pitch the robot rests at on the floor, radians (positive about body y). */
    double rest_pitch_rad = 0.0;
    std::vector<AprilTagMountConfiguration> tags;

    AprilTagKeypointModelConfiguration() { type = "AprilTagKeypointModel"; }

    void parse_fields(ConfigParser &parser) override {
        tag_size_m = parser.get_optional_double("tag_size_m", tag_size_m);
        const auto ids = parser.get_optional_vector<int64_t>("robot_tag_ids", {41, 76});
        robot_tag_ids.assign(ids.begin(), ids.end());
        roi_margin_px = static_cast<int>(parser.get_optional_int("roi_margin_px", roi_margin_px));
        max_reprojection_error_px =
            parser.get_optional_double("max_reprojection_error_px", max_reprojection_error_px);
        front_keypoint_m = parse_vector3(parser, "front_keypoint_m", front_keypoint_m);
        back_keypoint_m = parse_vector3(parser, "back_keypoint_m", back_keypoint_m);
        axle_height_m = parser.get_optional_double("axle_height_m", axle_height_m);
        rest_pitch_rad = parser.get_optional_double("rest_pitch_rad", rest_pitch_rad);
        parse_tags(parser);

        if (tag_size_m <= 0.0) {
            throw ConfigValidationError("tag_size_m must be > 0 in section [keypoint_model]");
        }
        if (roi_margin_px < 0) {
            throw ConfigValidationError("roi_margin_px must be >= 0 in section [keypoint_model]");
        }
        for (int id : robot_tag_ids) {
            const bool mounted = std::any_of(tags.begin(), tags.end(),
                                             [id](const auto &tag) { return tag.id == id; });
            if (!mounted) {
                throw ConfigValidationError("robot tag " + std::to_string(id) +
                                            " has no [[keypoint_model.tags]] entry");
            }
        }
    }

   private:
    static std::array<double, 3> parse_vector3(ConfigParser &parser, const std::string &key,
                                               const std::array<double, 3> &fallback) {
        const auto values = parser.get_optional_vector<double>(key, {});
        if (values.empty()) return fallback;
        if (values.size() != 3) {
            throw ConfigValidationError("'" + key + "' must have 3 elements in [keypoint_model]");
        }
        return {values[0], values[1], values[2]};
    }

    void parse_tags(ConfigParser &parser) {
        const toml::array *entries = parser.get_array("tags");
        if (!entries) return;
        tags.clear();
        for (const auto &node : *entries) {
            const toml::table *entry = node.as_table();
            if (!entry) throw ConfigValidationError("each [[keypoint_model.tags]] must be a table");
            ConfigParser tag_parser(*entry, "keypoint_model.tags");
            AprilTagMountConfiguration tag;
            tag.id = static_cast<int>(tag_parser.get_required_int("id"));
            const auto translation = tag_parser.get_required_vector<double>("translation_m");
            const auto rotation = tag_parser.get_required_vector<double>("rotation");
            tag_parser.validate_no_extra_fields();
            if (translation.size() != 3 || rotation.size() != 9) {
                throw ConfigValidationError("tag " + std::to_string(tag.id) +
                                            ": translation_m needs 3 values and rotation 9");
            }
            std::copy(translation.begin(), translation.end(), tag.translation_m.begin());
            std::copy(rotation.begin(), rotation.end(), tag.rotation.begin());
            validate_rotation(tag);
            tags.push_back(tag);
        }
    }

    /** A rotation copied by hand with a transposed or dropped element still parses; this makes
     *  it fail here instead of as a skewed pose. */
    static void validate_rotation(const AprilTagMountConfiguration &tag) {
        const auto &r = tag.rotation;
        constexpr double kTolerance = 1e-3;
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) {
                double dot = 0.0;
                for (int k = 0; k < 3; ++k) dot += r[3 * k + a] * r[3 * k + b];
                if (std::abs(dot - (a == b ? 1.0 : 0.0)) > kTolerance) {
                    throw ConfigValidationError("tag " + std::to_string(tag.id) +
                                                ": rotation is not orthonormal");
                }
            }
        }
        const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                           r[2] * (r[3] * r[7] - r[4] * r[6]);
        if (det < 0.0) {
            throw ConfigValidationError("tag " + std::to_string(tag.id) +
                                        ": rotation is a reflection (determinant -1)");
        }
    }
};

/** `sink` and `mcap_recorder` may be null; only models that publish their own topics use them. */
std::shared_ptr<KeypointModelInterface> make_keypoint_model(
    const KeypointModelConfiguration &config, std::shared_ptr<VizSink> sink,
    std::shared_ptr<McapRecorder> mcap_recorder);
std::unique_ptr<KeypointModelConfiguration> parse_keypoint_model_config(ConfigParser &parser);
std::unique_ptr<KeypointModelConfiguration> load_keypoint_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
