#pragma once

#include <set>

#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "enums/keypoint_label.hpp"
#include "enums/label.hpp"
#include "robot_blob_model/robot_blob_model_interface.hpp"

namespace auto_battlebot {
struct RobotBlobModelConfiguration {
    std::string type;
    virtual ~RobotBlobModelConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopRobotBlobModelConfiguration : public RobotBlobModelConfiguration {
    NoopRobotBlobModelConfiguration() { type = "NoopRobotBlobModel"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct YoloSegRobotBlobModelConfiguration : public RobotBlobModelConfiguration {
    std::string model_path;
    float confidence_threshold = 0.5f;
    float iou_threshold = 0.45f;
    float mask_threshold = 0.5f;
    float letterbox_padding = 0.1f;
    int image_size = 640;
    int max_detections = 32;
    bool debug_visualization = false;
    std::vector<Label> label_indices;
    std::vector<Label> their_robot_labels;
    std::vector<Label> neutral_robot_labels;
    std::vector<Label> field_labels;

    YoloSegRobotBlobModelConfiguration() { type = "YoloSegRobotBlobModel"; }

    void parse_fields(ConfigParser &parser) override;
};

std::shared_ptr<RobotBlobModelInterface> make_robot_blob_model(
    const RobotBlobModelConfiguration &config);
std::unique_ptr<RobotBlobModelConfiguration> parse_robot_blob_model_config(ConfigParser &parser);
std::unique_ptr<RobotBlobModelConfiguration> load_robot_blob_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot

