#pragma once

#include "data_structures.hpp"
#include "field_model/field_model_interface.hpp"
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

    std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config);
    std::unique_ptr<FieldModelConfiguration> parse_field_model_config(ConfigParser &parser);
} // namespace auto_battlebot
