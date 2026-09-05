#pragma once

#include "robot_blob_model/robot_blob_model_interface.hpp"

namespace auto_battlebot {
class NoopRobotBlobModel : public RobotBlobModelInterface {
   public:
    bool initialize() override { return true; }

    ModelResultStamped update([[maybe_unused]] RgbImage image) override {
        return ModelResultStamped{};
    }
};
}  // namespace auto_battlebot
