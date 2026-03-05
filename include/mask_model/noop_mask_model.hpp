#pragma once

#include "mask_model/mask_model_interface.hpp"

namespace auto_battlebot {
class NoopMaskModel : public MaskModelInterface {
   public:
    bool initialize() override { return true; }

    MaskStamped update([[maybe_unused]] RgbImage image) override { return MaskStamped{}; }
};

}  // namespace auto_battlebot
