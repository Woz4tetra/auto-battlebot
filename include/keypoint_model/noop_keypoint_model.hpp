#pragma once

#include "keypoint_model/keypoint_model_interface.hpp"

namespace auto_battlebot {
class NoopKeypointModel : public KeypointModelInterface {
   public:
    bool initialize() override { return true; }

    ModelResultStamped update([[maybe_unused]] RgbImage image) override {
        return ModelResultStamped{};
    }
};

}  // namespace auto_battlebot
