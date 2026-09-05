#pragma once

#include <limits>
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
    /** Height above the field plane in metres, measured from depth by KeypointHeightGate. NaN
     *  when nothing measured it, in which case consumers fall back to the configured per-label
     *  constant. Projecting with the measured value removes the assumption that every robot
     *  stands exactly `keypoint_height_meters` tall. */
    double height_above_plane = std::numeric_limits<double>::quiet_NaN();
};

/** Detection box in original-image pixels. */
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

/** One model's output for one frame: the keypoints it produced and the boxes they came from. */
struct ModelResultStamped {
    Header header;
    std::vector<Keypoint> keypoints;
    /** Box per detection, indexed by Keypoint::detection_index. Empty for models that do not
     *  produce boxes, and entries exist for detections that yielded no keypoints, so the index
     *  stays aligned with what the detector emitted. */
    std::vector<BoundingBox> boxes;

    /** Box for a keypoint's detection, or nullptr when this model carries none. */
    const BoundingBox *box_for(int detection_index) const {
        if (detection_index < 0 || static_cast<size_t>(detection_index) >= boxes.size()) {
            return nullptr;
        }
        return &boxes[static_cast<size_t>(detection_index)];
    }
};

}  // namespace auto_battlebot
