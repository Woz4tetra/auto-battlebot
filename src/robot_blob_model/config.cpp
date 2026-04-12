#include "robot_blob_model/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include "config/config_cast.hpp"
#include "robot_blob_model/noop_robot_blob_model.hpp"
#include "robot_blob_model/yolo_seg_robot_blob_model.hpp"

namespace auto_battlebot {
namespace {
std::vector<Label> parse_label_list(ConfigParser &parser, const std::string &field_name) {
    std::vector<std::string> label_names = parser.get_optional_vector<std::string>(field_name);
    std::vector<Label> labels;
    labels.reserve(label_names.size());
    for (const std::string &name : label_names) {
        auto label = magic_enum::enum_cast<Label>(name);
        if (!label.has_value()) {
            throw std::invalid_argument("Invalid value: '" + name + "' for field '" + field_name +
                                        "'");
        }
        labels.push_back(label.value());
    }
    return labels;
}
}  // namespace

void YoloSegRobotBlobModelConfiguration::parse_fields(ConfigParser &parser) {
    model_path = parser.get_required_string("model_path");
    confidence_threshold = parser.get_optional_double("confidence_threshold", confidence_threshold);
    iou_threshold = parser.get_optional_double("iou_threshold", iou_threshold);
    mask_threshold = parser.get_optional_double("mask_threshold", mask_threshold);
    letterbox_padding = parser.get_optional_double("letterbox_padding", letterbox_padding);
    image_size = parser.get_optional_int("image_size", image_size);
    max_detections = parser.get_optional_int("max_detections", max_detections);
    debug_visualization = parser.get_optional_bool("debug_visualization", debug_visualization);
    label_indices = parse_label_list(parser, "label_indices");
    their_robot_labels = parse_label_list(parser, "their_robot_labels");
    neutral_robot_labels = parse_label_list(parser, "neutral_robot_labels");
    field_labels = parse_label_list(parser, "field_labels");
    if (label_indices.empty()) {
        throw ConfigValidationError(
            "Field 'label_indices' must not be empty in section "
            "[robot_mask_model]");
    }
    parser.validate_no_extra_fields();
}

REGISTER_CONFIG(RobotBlobModelConfiguration, NoopRobotBlobModelConfiguration, "NoopRobotBlobModel")
REGISTER_CONFIG(RobotBlobModelConfiguration, YoloSegRobotBlobModelConfiguration,
                "YoloSegRobotBlobModel")

std::unique_ptr<RobotBlobModelConfiguration> parse_robot_blob_model_config(ConfigParser &parser) {
    return ConfigFactory<RobotBlobModelConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<RobotBlobModelConfiguration> load_robot_blob_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["robot_mask_model"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [robot_mask_model]");
    }
    ConfigParser parser(*section, "robot_mask_model");
    auto config = parse_robot_blob_model_config(parser);
    parsed_sections.push_back("robot_mask_model");
    return config;
}

std::shared_ptr<RobotBlobModelInterface> make_robot_blob_model(
    const RobotBlobModelConfiguration &config) {
    spdlog::info("Selected {} for RobotBlobModel", config.type);
    if (config.type == "NoopRobotBlobModel") {
        return std::make_shared<NoopRobotBlobModel>();
    } else if (config.type == "YoloSegRobotBlobModel") {
        return std::make_shared<YoloSegRobotBlobModel>(
            config_cast<YoloSegRobotBlobModelConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load RobotBlobModel of type " + config.type);
}
}  // namespace auto_battlebot
