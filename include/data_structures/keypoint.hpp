#pragma once

#include <string>
#include <vector>
#include "header.hpp"
#include "camera.hpp"
#include "enums.hpp"

namespace auto_battlebot
{
    struct Keypoint
    {
        Label label = Label::EMPTY;
        KeypointLabel keypoint_label = KeypointLabel::EMPTY;
        double x = 0.0f;
        double y = 0.0f;
    };

    struct KeypointsStamped
    {
        Header header;
        std::vector<Keypoint> keypoints;
    };

} // namespace auto_battlebot
