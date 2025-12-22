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

    struct PointCloudFieldFilterConfiguration : public FieldFilterConfiguration
    {
        double distance_threshold = 0.1;
        bool local_visualize_debug = false;
        double depth_units_per_meter = 1000.0;

        PointCloudFieldFilterConfiguration()
        {
            type = "PointCloudFieldFilter";
        }

        // clang-format off
        PARSE_CONFIG_FIELDS(
            PARSE_FIELD_DOUBLE(distance_threshold)
            PARSE_FIELD_BOOL(local_visualize_debug)
            PARSE_FIELD_DOUBLE(depth_units_per_meter)
        )
        // clang-format on
    };

    std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config);
    std::unique_ptr<FieldFilterConfiguration> parse_field_filter_config(ConfigParser &parser);
} // namespace auto_battlebot
