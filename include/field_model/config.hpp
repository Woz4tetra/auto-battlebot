#pragma once

#include "data_structures.hpp"
#include "field_model/field_model_interface.hpp"

namespace auto_battlebot
{
    struct FieldModelConfiguration
    {
        std::string type;
    };

    struct NoopFieldModelConfiguration : public FieldModelConfiguration
    {
        NoopFieldModelConfiguration()
        {
            type = "NoopFieldModel";
        }
    };

    std::shared_ptr<FieldModelInterface> make_field_model(const FieldModelConfiguration &config);
} // namespace auto_battlebot
