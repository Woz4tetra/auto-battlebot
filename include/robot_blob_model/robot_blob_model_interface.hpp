#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class RobotBlobModelInterface {
   public:
    virtual ~RobotBlobModelInterface() = default;
    virtual bool initialize() = 0;
    virtual ModelResultStamped update(RgbImage image) = 0;
    virtual DetectionsStamped last_detections_original_px() const { return {}; }
};
}  // namespace auto_battlebot
