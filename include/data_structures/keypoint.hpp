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
    double confidence = 1.0;
    int detection_index = 0;
    /** Height above the field plane in metres, measured from depth by KeypointHeightGate. NaN
     *  when nothing measured it, in which case consumers fall back to the configured per-label
     *  constant. Projecting with the measured value removes the assumption that every robot
     *  stands exactly `keypoint_height_meters` tall. */
    double height_above_plane = std::numeric_limits<double>::quiet_NaN();
    /** How far the robot's reference point sits ahead of its front/back keypoint midpoint, in
     *  metres along its heading. The reference point is our robot's axle, the body origin the
     *  MuJoCo plant and the RL policy use, and an opponent's footprint centre. Set by the model
     *  that knows where its keypoints sit on the robot; 0 for models that place them symmetrically
     *  about the reference point. */
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
    /** Box per detection, indexed by Keypoint::detection_index. Empty for models that do not
     *  produce boxes, and entries exist for detections that yielded no keypoints, so the index
     *  stays aligned with what the detector emitted. */
    std::vector<BoundingBox> boxes;

    const BoundingBox *box_for(int detection_index) const {
        if (detection_index < 0 || static_cast<size_t>(detection_index) >= boxes.size()) {
            return nullptr;
        }
        return &boxes[static_cast<size_t>(detection_index)];
    }
};

}  // namespace auto_battlebot
