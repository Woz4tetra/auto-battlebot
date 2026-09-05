#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class KeypointModelInterface {
   public:
    virtual ~KeypointModelInterface() = default;
    virtual bool initialize() = 0;
    virtual ModelResultStamped update(RgbImage image) = 0;
    // Raw detections (boxes + model keypoints) from the most recent update() call, in
    // original-image pixels. Default is empty for models that don't expose them.
    virtual DetectionsStamped last_detections() const { return {}; }
};

}  // namespace auto_battlebot
