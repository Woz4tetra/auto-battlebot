#pragma once

#include <optional>
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
    double confidence = 1.0;
    int detection_index = 0;
    std::optional<double> height_above_plane_m = std::nullopt;
    double reference_offset_m = 0.0;
};

struct BoundingBox {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;

    double width() const { return x2 - x1; }
    double height() const { return y2 - y1; }
    double center_x() const { return 0.5 * (x1 + x2); }
    double center_y() const { return 0.5 * (y1 + y2); }
};

struct ModelResultStamped {
    Header header;
    std::vector<Keypoint> keypoints;
    std::vector<BoundingBox> boxes;

    const BoundingBox *box_for(int detection_index) const {
        if (detection_index < 0 || static_cast<size_t>(detection_index) >= boxes.size()) {
            return nullptr;
        }
        return &boxes[static_cast<size_t>(detection_index)];
    }
};

}  // namespace auto_battlebot
