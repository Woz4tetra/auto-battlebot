#include "keypoint_filter/height_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "enum_to_string_lower.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {
constexpr double kEpsilon = 1e-6;
using GroupKey = std::pair<Label, int>;

double percentile_of(std::vector<double> &values, double fraction) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    const size_t index =
        std::min(values.size() - 1, static_cast<size_t>(fraction * (values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<long>(index), values.end());
    return values[index];
}

double inscribed_circle_radius(const BoundingBox &box) {
    return 0.5 * std::min(box.width(), box.height());
}

int sample_stride_for_budget(long area, int max_circle_samples, double ring_outer_scale) {
    const double budget =
        static_cast<double>(max_circle_samples) * (ring_outer_scale * ring_outer_scale);
    return std::max(1, static_cast<int>(std::sqrt(static_cast<double>(area) / budget)));
}
}  // namespace

KeypointHeightGate::KeypointHeightGate(const KeypointHeightGateConfiguration &config)
    : config_(config), diagnostics_logger_(DiagnosticsLogger::get_logger("keypoint_height_gate")) {}

double KeypointHeightGate::height_at(const cv::Mat &depth, const CameraInfo &camera_info,
                                     const Plane &plane, int col, int row) const {
    const float z = depth.ptr<float>(row)[col];
    if (!std::isfinite(z) || z <= 0.0f) return std::numeric_limits<double>::quiet_NaN();

    Eigen::Vector3d ray;
    if (!pixel_to_camera_ray(camera_info, col, row, ray)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Eigen::Vector3d point = ray * static_cast<double>(z);
    return plane.orientation * (point - plane.center).dot(plane.normal);
}

double KeypointHeightGate::height_at_keypoint(const cv::Mat &depth, const CameraInfo &camera_info,
                                              const Plane &plane, double x, double y) const {
    const int cx = static_cast<int>(std::lround(x));
    const int cy = static_cast<int>(std::lround(y));
    const int r = config_.sample_radius_px;

    const int x0 = std::max(0, cx - r);
    const int x1 = std::min(depth.cols - 1, cx + r);
    const int y0 = std::max(0, cy - r);
    const int y1 = std::min(depth.rows - 1, cy + r);
    if (x0 > x1 || y0 > y1) return std::numeric_limits<double>::quiet_NaN();

    std::vector<double> heights;
    heights.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
    for (int row = y0; row <= y1; ++row) {
        for (int col = x0; col <= x1; ++col) {
            const double height = height_at(depth, camera_info, plane, col, row);
            if (std::isfinite(height)) heights.push_back(height);
        }
    }
    if (static_cast<int>(heights.size()) < config_.min_valid_samples) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return percentile_of(heights, 0.5);
}

KeypointHeightGate::Relief KeypointHeightGate::measure_relief(
    const cv::Mat &depth, const CameraInfo &camera_info, const Plane &plane,
    const ModelResultStamped &model_results, const std::vector<size_t> &indices) const {
    Relief relief;
    if (indices.empty()) return relief;
    const BoundingBox *box =
        model_results.box_for(model_results.keypoints[indices[0]].detection_index);
    if (box == nullptr) return relief;

    const double radius = inscribed_circle_radius(*box);
    if (!(radius > 0.0)) return relief;
    const double centre_x = box->center_x();
    const double centre_y = box->center_y();

    const double ring_inner = radius * config_.ring_inner_scale;
    const double ring_outer = radius * config_.ring_outer_scale;

    const int x0 = std::max(0, static_cast<int>(std::floor(centre_x - ring_outer)));
    const int x1 = std::min(depth.cols - 1, static_cast<int>(std::ceil(centre_x + ring_outer)));
    const int y0 = std::max(0, static_cast<int>(std::floor(centre_y - ring_outer)));
    const int y1 = std::min(depth.rows - 1, static_cast<int>(std::ceil(centre_y + ring_outer)));
    if (x0 > x1 || y0 > y1) return relief;

    const long area = static_cast<long>(x1 - x0 + 1) * static_cast<long>(y1 - y0 + 1);
    const int stride =
        sample_stride_for_budget(area, config_.max_circle_samples, config_.ring_outer_scale);

    const double inner_squared = radius * radius;
    const double ring_inner_squared = ring_inner * ring_inner;
    const double ring_outer_squared = ring_outer * ring_outer;

    std::vector<double> inside;
    std::vector<double> around;
    for (int row = y0; row <= y1; row += stride) {
        const double dy = row - centre_y;
        for (int col = x0; col <= x1; col += stride) {
            const double dx = col - centre_x;
            const double distance_squared = dx * dx + dy * dy;
            if (distance_squared > ring_outer_squared) continue;
            const bool in_circle = distance_squared <= inner_squared;
            const bool in_ring = distance_squared >= ring_inner_squared;
            if (!in_circle && !in_ring) continue;
            const double height = height_at(depth, camera_info, plane, col, row);
            if (!std::isfinite(height)) continue;
            (in_circle ? inside : around).push_back(height);
        }
    }

    if (static_cast<int>(inside.size()) < config_.min_valid_samples) return relief;
    if (static_cast<int>(around.size()) < config_.min_valid_samples) return relief;

    relief.top = percentile_of(inside, config_.top_percentile);
    const double local_floor = percentile_of(around, config_.floor_percentile);
    if (!std::isfinite(relief.top) || !std::isfinite(local_floor)) return relief;
    relief.elevation = relief.top - local_floor;
    relief.valid = true;
    return relief;
}

ModelResultStamped KeypointHeightGate::filter(const ModelResultStamped &model_results,
                                              const DepthImage &depth,
                                              const CameraInfo &camera_info,
                                              const FieldDescription &field) {
    stats_ = Stats{};
    if (model_results.keypoints.empty()) return model_results;

    if (depth.image.empty() || depth.image.type() != CV_32FC1) return model_results;

    Plane plane;
    plane.center = Eigen::Vector3d::Zero();
    plane.normal = Eigen::Vector3d::UnitZ();
    if (!transform_to_plane_center_normal(field.tf_camera_from_fieldcenter, plane.center,
                                          plane.normal)) {
        return model_results;
    }
    const double signed_plane_offset = plane.normal.dot(plane.center);
    if (std::abs(signed_plane_offset) <= kEpsilon) return model_results;
    plane.orientation = (signed_plane_offset > 0.0) ? -1.0 : 1.0;

    std::map<GroupKey, std::vector<size_t>> groups;
    for (size_t i = 0; i < model_results.keypoints.size(); ++i) {
        const Keypoint &kp = model_results.keypoints[i];
        groups[{kp.label, kp.detection_index}].push_back(i);
    }

    ModelResultStamped output;
    output.header = model_results.header;
    output.boxes = model_results.boxes;
    output.keypoints.reserve(model_results.keypoints.size());

    for (const auto &[group_key, indices] : groups) {
        stats_.groups_in++;

        const Relief relief =
            measure_relief(depth.image, camera_info, plane, model_results, indices);

        std::vector<double> keypoint_heights;
        keypoint_heights.reserve(indices.size());
        for (size_t index : indices) {
            const Keypoint &kp = model_results.keypoints[index];
            keypoint_heights.push_back(
                height_at_keypoint(depth.image, camera_info, plane, kp.x, kp.y));
        }

        if (!relief.valid) {
            stats_.abstained++;
            stats_.groups_passed++;
            for (size_t n = 0; n < indices.size(); ++n) {
                Keypoint kp = model_results.keypoints[indices[n]];
                kp.height_above_plane = keypoint_heights[n];
                output.keypoints.push_back(kp);
            }
            continue;
        }

        const bool too_low = relief.elevation < config_.min_elevation_meters;
        const bool too_high = relief.top > config_.max_height_meters;
        if (config_.reject_enable && (too_low || too_high)) {
            (too_low ? stats_.rejected_low : stats_.rejected_high)++;

            DiagnosticsData rejection;
            rejection["label"] = enum_to_string_lower(group_key.first);
            rejection["detection_index"] = group_key.second;
            rejection["elevation_m"] = relief.elevation;
            rejection["top_m"] = relief.top;
            diagnostics_logger_->debug(too_low ? "rejected_low" : "rejected_high", rejection);
            continue;
        }

        stats_.groups_passed++;
        for (size_t n = 0; n < indices.size(); ++n) {
            Keypoint kp = model_results.keypoints[indices[n]];
            kp.height_above_plane =
                std::isfinite(keypoint_heights[n]) ? keypoint_heights[n] : relief.top;
            output.keypoints.push_back(kp);
        }
    }

    DiagnosticsData summary;
    summary["groups_in"] = stats_.groups_in;
    summary["groups_passed"] = stats_.groups_passed;
    summary["rejected_low"] = stats_.rejected_low;
    summary["rejected_high"] = stats_.rejected_high;
    summary["abstained"] = stats_.abstained;
    diagnostics_logger_->debug(summary);

    return output;
}

}  // namespace auto_battlebot
