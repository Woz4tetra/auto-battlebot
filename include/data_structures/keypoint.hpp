#pragma once

#include <string>
#include <vector>
#include "header.hpp"
#include "camera.hpp"

namespace auto_battlebot {

struct Keypoint {
    std::string label;
    std::string keypoint_label;
    float x;
    float y;
};

struct KeypointsStamped {
    Header header;
    std::vector<Keypoint> keypoints;
    CameraInfo camera_info;
};

}  // namespace auto_battlebot
