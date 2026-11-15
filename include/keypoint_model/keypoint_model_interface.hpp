#pragma once

#include "data_structures.hpp"

namespace auto_battlebot {
class KeypointModelInterface {
   public:
    virtual ~KeypointModelInterface() = default;
    virtual bool initialize() = 0;
    virtual ModelResultStamped update(RgbImage image, const CameraInfo &camera_info) = 0;
    virtual DetectionsStamped last_detections() const { return {}; }
};

}  // namespace auto_battlebot
