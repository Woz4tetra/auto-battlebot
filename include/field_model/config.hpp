#pragma once

#include "data_structures.hpp"

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
} // namespace auto_battlebot
