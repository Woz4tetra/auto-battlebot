#pragma once

#include "data_structures.hpp"

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
} // namespace auto_battlebot
