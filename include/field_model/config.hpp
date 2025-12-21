#pragma once

#include "data_structures.hpp"
#include "field_model/field_model_interface.hpp"
#include "enums/deeplab_model_type.hpp"
#include "config_factory.hpp"

namespace auto_battlebot
{
    struct FieldModelConfiguration
    {
        std::string type;
        virtual ~FieldModelConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopFieldModelConfiguration : public FieldModelConfiguration
    {
        NoopFieldModelConfiguration()
        {
            type = "NoopFieldModel";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    struct DeepLabFieldModelConfiguration : public FieldModelConfiguration
    {
        std::string model_path;
        DeepLabModelType model_type = DeepLabModelType::DeepLabV3;
        int image_size = 512;

        DeepLabFieldModelConfiguration()
        {
            type = "DeepLabFieldModel";
        }

        PARSE_CONFIG_FIELDS(
            PARSE_FIELD_STRING_REQUIRED(model_path)
                PARSE_ENUM(model_type, DeepLabModelType)
                    PARSE_FIELD(image_size))
    };

    std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config);
    std::unique_ptr<FieldModelConfiguration> parse_field_model_config(ConfigParser &parser);
} // namespace auto_battlebot
