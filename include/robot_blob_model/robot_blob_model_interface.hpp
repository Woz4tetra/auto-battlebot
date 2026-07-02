#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class RobotBlobModelInterface {
   public:
    virtual ~RobotBlobModelInterface() = default;
    virtual bool initialize() = 0;
    virtual KeypointsStamped update(RgbImage image) = 0;
    // Raw detections from the most recent update() call, in original-image pixels.
    // Default is empty for models that don't produce boxed detections.
    virtual DetectionsStamped last_detections() const { return {}; }
};
}  // namespace auto_battlebot
