#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    class FieldModelInterface
    {
    public:
        virtual ~FieldModelInterface() = default;
        virtual bool initialize() = 0;
        virtual FieldMaskStamped update(RgbImage image) = 0;
    };

} // namespace auto_battlebot
