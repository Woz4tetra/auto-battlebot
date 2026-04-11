#pragma once

#include <opencv2/core.hpp>

#include "data_structures/keypoint.hpp"
#include "enums/keypoint_label.hpp"
#include "enums/label.hpp"

namespace auto_battlebot {
struct MidlineSegment {
    cv::Point2f start{};
    cv::Point2f end{};
    float length = 0.0f;
    bool valid = false;
};

MidlineSegment find_longest_rectangle_midline(const cv::Mat &instance_mask);

MidlineSegment bbox_to_midline(float x1, float y1, float x2, float y2);

void append_midline_keypoints(const MidlineSegment &midline, Label label,
                              KeypointLabel endpoint_a_label, KeypointLabel endpoint_b_label,
                              double confidence, int detection_index,
                              std::vector<Keypoint> &output);
}  // namespace auto_battlebot

