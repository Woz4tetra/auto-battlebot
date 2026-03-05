#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class MaskModelInterface {
   public:
    virtual ~MaskModelInterface() = default;
    virtual bool initialize() = 0;
    virtual MaskStamped update(RgbImage image) = 0;
};

}  // namespace auto_battlebot
