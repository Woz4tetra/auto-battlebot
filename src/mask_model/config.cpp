#include "mask_model/config.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <magic_enum.hpp>

#include "config/config_parser.hpp"
#include "mask_model/deeplab_mask_model.hpp"
#include "mask_model/noop_mask_model.hpp"
#include "mask_model/yolo_seg_mask_model.hpp"

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

void YoloSegMaskModelConfiguration::parse_fields(ConfigParser &parser) {
    model_path = parser.get_required_string("model_path");
    label_indices = parse_label_list(parser, "label_indices");
    if (label_indices.empty()) {
        throw ConfigValidationError(
            "Field 'label_indices' must not be empty in section [field_model]");
    }

    std::string output_label_name =
        parser.get_optional_string("output_label", magic_enum::enum_name(output_label).data());
    auto parsed_output_label = magic_enum::enum_cast<Label>(output_label_name);
    if (!parsed_output_label.has_value()) {
        throw std::invalid_argument("Invalid value: '" + output_label_name +
                                    "' for field 'output_label'");
    }
    output_label = parsed_output_label.value();

    confidence_threshold = parser.get_optional_double("confidence_threshold", confidence_threshold);
    iou_threshold = parser.get_optional_double("iou_threshold", iou_threshold);
    mask_threshold = parser.get_optional_double("mask_threshold", mask_threshold);
    letterbox_padding = parser.get_optional_double("letterbox_padding", letterbox_padding);
    image_size = parser.get_optional_int("image_size", image_size);
    max_detections = parser.get_optional_int("max_detections", max_detections);
    debug_visualization = parser.get_optional_bool("debug_visualization", debug_visualization);
    parser.validate_no_extra_fields();
}

// Automatic registration of config types
REGISTER_CONFIG(MaskModelConfiguration, NoopMaskModelConfiguration, "NoopMaskModel")
REGISTER_CONFIG(MaskModelConfiguration, DeepLabMaskModelConfiguration, "DeepLabMaskModel")
REGISTER_CONFIG(MaskModelConfiguration, YoloSegMaskModelConfiguration, "YoloSegMaskModel")

std::unique_ptr<MaskModelConfiguration> parse_mask_model_config(ConfigParser &parser) {
    return ConfigFactory<MaskModelConfiguration>::instance().create_and_parse(parser);
}

std::unique_ptr<MaskModelConfiguration> load_field_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["field_model"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [field_model]");
    }
    ConfigParser parser(*section, "field_model");
    auto config = parse_mask_model_config(parser);
    parsed_sections.push_back("field_model");
    return config;
}

std::unique_ptr<MaskModelConfiguration> load_robot_mask_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections) {
    auto section = toml_data["robot_mask_model"].as_table();
    if (!section) {
        throw ConfigValidationError("Missing required section [robot_mask_model]");
    }
    ConfigParser parser(*section, "robot_mask_model");
    auto config = parse_mask_model_config(parser);
    parsed_sections.push_back("robot_mask_model");
    return config;
}

std::shared_ptr<MaskModelInterface> make_mask_model(const MaskModelConfiguration &config) {
    spdlog::info("Selected {} for MaskModel", config.type);
    if (config.type == "NoopMaskModel") {
        return std::make_shared<NoopMaskModel>();
    } else if (config.type == "DeepLabMaskModel") {
        return std::make_shared<DeepLabMaskModel>(
            config_cast<DeepLabMaskModelConfiguration>(config));
    } else if (config.type == "YoloSegMaskModel") {
        return std::make_shared<YoloSegMaskModel>(
            config_cast<YoloSegMaskModelConfiguration>(config));
    }
    throw std::invalid_argument("Failed to load MaskModel of type " + config.type);
}
}  // namespace auto_battlebot
