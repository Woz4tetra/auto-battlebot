#pragma once

#include <vector>

#include "data_structures/header.hpp"
#include "enums/label.hpp"

namespace auto_battlebot {
// One model keypoint (e.g. front/back) in original-image pixels, ordered per the
// model's kpt layout.
struct DetectionKeypoint {
    double x = 0.0;
    double y = 0.0;
    double confidence = 0.0;
};

// Raw detector output in original-image pixel coordinates, captured before the
// blob/keypoint collapse so offline tools can score the model itself.
struct Detection2D {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
    double confidence = 0.0;
    int class_id = 0;
    Label label = Label::EMPTY;
    std::vector<DetectionKeypoint> keypoints;  // empty for box-only models
};

struct DetectionsStamped {
    Header header;
    int image_width = 0;
    int image_height = 0;
    std::vector<Detection2D> detections;
};

}  // namespace auto_battlebot
