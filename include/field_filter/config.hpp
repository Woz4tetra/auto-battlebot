#pragma once

#include "data_structures.hpp"
#include "field_filter/field_filter_interface.hpp"
#include "config_factory.hpp"

namespace auto_battlebot
{
    struct FieldFilterConfiguration
    {
        std::string type;
        virtual ~FieldFilterConfiguration() = default;
        virtual void parse_fields([[maybe_unused]] ConfigParser &parser) {}
    };

    struct NoopFieldFilterConfiguration : public FieldFilterConfiguration
    {
        NoopFieldFilterConfiguration()
        {
            type = "NoopFieldFilter";
        }

        PARSE_CONFIG_FIELDS(
            // No additional fields
        )
    };

    std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config);
    std::unique_ptr<FieldFilterConfiguration> parse_field_filter_config(ConfigParser &parser);
} // namespace auto_battlebot
