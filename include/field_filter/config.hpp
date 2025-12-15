#pragma once

#include "data_structures.hpp"
#include "field_filter/field_filter_interface.hpp"

namespace auto_battlebot
{
    struct FieldFilterConfiguration
    {
        std::string type;
    };

    struct NoopFieldFilterConfiguration : public FieldFilterConfiguration
    {
        NoopFieldFilterConfiguration()
        {
            type = "NoopFieldFilter";
        }
    };

    std::shared_ptr<FieldFilterInterface> make_field_filter(const FieldFilterConfiguration &config);
} // namespace auto_battlebot
