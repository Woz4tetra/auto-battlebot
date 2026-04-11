#pragma once

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "enums/deeplab_model_type.hpp"
#include "mask_model/mask_model_interface.hpp"

namespace auto_battlebot {
struct MaskModelConfiguration {
    std::string type;
    virtual ~MaskModelConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopMaskModelConfiguration : public MaskModelConfiguration {
    NoopMaskModelConfiguration() { type = "NoopMaskModel"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct DeepLabMaskModelConfiguration : public MaskModelConfiguration {
    std::string model_path;
    DeepLabModelType model_type = DeepLabModelType::DeepLabV3;
    Label output_label = Label::FIELD;

    DeepLabMaskModelConfiguration() { type = "DeepLabMaskModel"; }

    // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD_STRING_REQUIRED(model_path)
            PARSE_ENUM(model_type, DeepLabModelType)
            PARSE_ENUM(output_label, Label))
    // clang-format on
};

struct YoloSegMaskModelConfiguration : public MaskModelConfiguration {
    std::string model_path;
    std::vector<Label> label_indices;
    Label output_label = Label::FIELD;
    float confidence_threshold = 0.5f;
    float iou_threshold = 0.45f;
    float mask_threshold = 0.5f;
    float letterbox_padding = 0.1f;
    int image_size = 640;
    int max_detections = 32;
    bool debug_visualization = false;

    YoloSegMaskModelConfiguration() { type = "YoloSegMaskModel"; }

    void parse_fields(ConfigParser &parser) override;
};

std::shared_ptr<MaskModelInterface> make_mask_model(const MaskModelConfiguration &config);
std::unique_ptr<MaskModelConfiguration> parse_mask_model_config(ConfigParser &parser);
std::unique_ptr<MaskModelConfiguration> load_field_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
std::unique_ptr<MaskModelConfiguration> load_robot_mask_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
