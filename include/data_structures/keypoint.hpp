#pragma once

#include <string>
#include <vector>

#include "camera.hpp"
#include "enums.hpp"
#include "header.hpp"

namespace auto_battlebot {
struct Keypoint {
    Label label = Label::EMPTY;
    KeypointLabel keypoint_label = KeypointLabel::EMPTY;
    double x = 0.0f;
    double y = 0.0f;
    /** Detection confidence (e.g. from YOLO row[4]). Used to sort and keep top-N per label. */
    double confidence = 1.0;
    /** Index of the detection this keypoint belongs to (for grouping keypoints by instance). */
    int detection_index = 0;
};

struct KeypointsStamped {
    Header header;
    std::vector<Keypoint> keypoints;
};

}  // namespace auto_battlebot
