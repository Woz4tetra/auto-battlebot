#pragma once

#include "field_model/field_model_interface.hpp"

namespace auto_battlebot
{
    class NoopFieldModel : public FieldModelInterface
    {
    public:
        bool initialize() override { return true; }

        FieldMaskStamped update(RgbImage image) override
        {
            return FieldMaskStamped{};
        }
    };

} // namespace auto_battlebot
