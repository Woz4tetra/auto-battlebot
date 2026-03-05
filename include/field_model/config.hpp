#pragma once

#include "config/config_cast.hpp"
#include "config/config_factory.hpp"
#include "config/config_parser.hpp"
#include "data_structures.hpp"
#include "enums/deeplab_model_type.hpp"
#include "field_model/field_model_interface.hpp"

namespace auto_battlebot {
struct FieldModelConfiguration {
    std::string type;
    virtual ~FieldModelConfiguration() = default;
    virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
};

struct NoopFieldModelConfiguration : public FieldModelConfiguration {
    NoopFieldModelConfiguration() { type = "NoopFieldModel"; }

    PARSE_CONFIG_FIELDS(
        // No additional fields
    )
};

struct DeepLabFieldModelConfiguration : public FieldModelConfiguration {
    std::string model_path;
    DeepLabModelType model_type = DeepLabModelType::DeepLabV3;

    DeepLabFieldModelConfiguration() { type = "DeepLabFieldModel"; }

    // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD_STRING_REQUIRED(model_path)
            PARSE_ENUM(model_type, DeepLabModelType))
    // clang-format on
};

std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config);
std::unique_ptr<FieldModelConfiguration> parse_field_model_config(ConfigParser &parser);
std::unique_ptr<FieldModelConfiguration> load_field_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
std::unique_ptr<FieldModelConfiguration> load_floor_model_from_toml(
    toml::table const &toml_data, std::vector<std::string> &parsed_sections);
}  // namespace auto_battlebot
