#include "robot_blob_model/rectangle_keypoint_helpers.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace auto_battlebot {
MidlineSegment find_longest_rectangle_midline(const cv::Mat &instance_mask) {
    MidlineSegment result;
    if (instance_mask.empty()) return result;

    cv::Mat binary;
    if (instance_mask.type() == CV_8UC1) {
        cv::threshold(instance_mask, binary, 0, 255, cv::THRESH_BINARY);
    } else {
        cv::Mat converted;
        instance_mask.convertTo(converted, CV_8UC1);
        cv::threshold(converted, binary, 0, 255, cv::THRESH_BINARY);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return result;

    auto best_it =
        std::max_element(contours.begin(), contours.end(),
                         [](const std::vector<cv::Point> &lhs, const std::vector<cv::Point> &rhs) {
                             return cv::contourArea(lhs) < cv::contourArea(rhs);
                         });
    if (best_it == contours.end() || best_it->size() < 3) return result;

    cv::RotatedRect rect = cv::minAreaRect(*best_it);
    cv::Point2f corners[4];
    rect.points(corners);

    const cv::Point2f mid01 = 0.5f * (corners[0] + corners[1]);
    const cv::Point2f mid12 = 0.5f * (corners[1] + corners[2]);
    const cv::Point2f mid23 = 0.5f * (corners[2] + corners[3]);
    const cv::Point2f mid30 = 0.5f * (corners[3] + corners[0]);

    const float len_a = static_cast<float>(cv::norm(mid01 - mid23));
    const float len_b = static_cast<float>(cv::norm(mid12 - mid30));
    if (len_a >= len_b) {
        result.start = mid01;
        result.end = mid23;
        result.length = len_a;
    } else {
        result.start = mid12;
        result.end = mid30;
        result.length = len_b;
    }
    result.valid = result.length > 0.0f;
    return result;
}

MidlineSegment bbox_to_midline(float x1, float y1, float x2, float y2) {
    MidlineSegment result;
    const float w = std::max(0.0f, x2 - x1);
    const float h = std::max(0.0f, y2 - y1);
    if (w <= 0.0f || h <= 0.0f) return result;

    const float cx = 0.5f * (x1 + x2);
    const float cy = 0.5f * (y1 + y2);
    if (w >= h) {
        result.start = cv::Point2f(x1, cy);
        result.end = cv::Point2f(x2, cy);
        result.length = w;
    } else {
        result.start = cv::Point2f(cx, y1);
        result.end = cv::Point2f(cx, y2);
        result.length = h;
    }
    result.valid = true;
    return result;
}

void append_midline_keypoints(const MidlineSegment &midline, Label label,
                              KeypointLabel endpoint_a_label, KeypointLabel endpoint_b_label,
                              double confidence, int detection_index,
                              std::vector<Keypoint> &output) {
    if (!midline.valid) return;

    Keypoint a;
    a.label = label;
    a.keypoint_label = endpoint_a_label;
    a.x = midline.start.x;
    a.y = midline.start.y;
    a.confidence = confidence;
    a.detection_index = detection_index;
    output.push_back(a);

    Keypoint b;
    b.label = label;
    b.keypoint_label = endpoint_b_label;
    b.x = midline.end.x;
    b.y = midline.end.y;
    b.confidence = confidence;
    b.detection_index = detection_index;
    output.push_back(b);
}
}  // namespace auto_battlebot

