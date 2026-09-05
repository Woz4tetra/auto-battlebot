#include "keypoint_filter/static_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

#include "enum_to_string_lower.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {
using GroupKey = std::pair<Label, int>;
}  // namespace

void StaticDetectionGate::Cluster::observe(const Eigen::Vector2d &position,
                                           const StaticDetectionGateConfiguration &config,
                                           double stamp) {
    const double deviation = (mean - position).norm();
    observations++;
    // Cap the averaging weight so both statistics keep tracking instead of going numb. Below the
    // window this is an exact running mean; above it, an exponential one with that time constant.
    const double weight =
        1.0 / static_cast<double>(std::min(observations, config.responsiveness_window));
    mean_deviation += (deviation - mean_deviation) * weight;
    mean += (position - mean) * weight;
    last_seen = stamp;
}

bool StaticDetectionGate::Cluster::is_static(const StaticDetectionGateConfiguration &config,
                                             double now) const {
    if (observations < config.min_observations) return false;
    if (mean_deviation > config.static_radius_meters) return false;
    return (now - first_seen) >= config.min_dwell_seconds;
}

StaticDetectionGate::StaticDetectionGate(const StaticDetectionGateConfiguration &config)
    : config_(config),
      diagnostics_logger_(DiagnosticsLogger::get_logger("static_detection_gate")) {}

void StaticDetectionGate::reset() { clusters_.clear(); }

ModelResultStamped StaticDetectionGate::filter(const ModelResultStamped &model_results,
                                               const CameraInfo &camera_info,
                                               const FieldDescription &field, double stamp) {
    stats_ = Stats{};
    if (!config_.enable) return model_results;

    // Clusters are held in field coordinates, so a valid field transform is what makes them
    // comparable across frames. Without one, pass everything.
    Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
    if (!transform_to_plane_center_normal(field.tf_camera_from_fieldcenter, plane_center,
                                          plane_normal)) {
        return model_results;
    }
    const Eigen::Matrix4d tf_field_from_camera =
        field.tf_camera_from_fieldcenter.tf.block<4, 4>(0, 0).inverse();

    // Drop clusters nothing has refreshed recently, so a graphic that leaves the frame does not
    // keep suppressing whatever later occupies its old spot.
    clusters_.erase(std::remove_if(clusters_.begin(), clusters_.end(),
                                   [&](const Cluster &cluster) {
                                       return (stamp - cluster.last_seen) > config_.forget_seconds;
                                   }),
                    clusters_.end());

    if (model_results.keypoints.empty()) {
        stats_.tracked_clusters = static_cast<int>(clusters_.size());
        return model_results;
    }

    std::map<GroupKey, std::vector<size_t>> groups;
    for (size_t i = 0; i < model_results.keypoints.size(); ++i) {
        const Keypoint &kp = model_results.keypoints[i];
        groups[{kp.label, kp.detection_index}].push_back(i);
    }

    ModelResultStamped output;
    output.header = model_results.header;
    // Carry the boxes through. detection_index indexes into them, so dropping them here would
    // leave the height gate downstream with nothing to sample inside.
    output.boxes = model_results.boxes;
    output.keypoints.reserve(model_results.keypoints.size());

    for (const auto &[group_key, indices] : groups) {
        stats_.groups_in++;

        // The group's field position is the centroid of its model_results projected onto the plane.
        // Using the measured height where the height gate supplied one keeps a tall robot's
        // footprint from being pushed outward by the flat-plane assumption.
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int projected = 0;
        for (size_t index : indices) {
            const Keypoint &kp = model_results.keypoints[index];
            const double height =
                std::isfinite(kp.height_above_plane) ? kp.height_above_plane : 0.0;
            Eigen::Vector3d point;
            if (!project_keypoint_onto_plane(kp, plane_center, plane_normal, camera_info, height,
                                             point)) {
                continue;
            }
            sum += point;
            projected++;
        }
        if (projected == 0) {
            // Cannot place this group on the field, so it cannot be judged. Pass it through.
            stats_.groups_passed++;
            for (size_t index : indices) output.keypoints.push_back(model_results.keypoints[index]);
            continue;
        }

        const Eigen::Vector3d camera_point = sum / static_cast<double>(projected);
        const Eigen::Vector3d field_point = transform_point(tf_field_from_camera, camera_point);
        const Eigen::Vector2d position(field_point.x(), field_point.y());

        // Match to the nearest tracked cluster within the match radius.
        Cluster *matched = nullptr;
        double best_distance = std::numeric_limits<double>::infinity();
        for (Cluster &cluster : clusters_) {
            const double distance = (cluster.mean - position).norm();
            if (distance < best_distance && distance <= config_.match_radius_meters) {
                best_distance = distance;
                matched = &cluster;
            }
        }

        if (matched == nullptr) {
            Cluster cluster;
            cluster.mean = position;
            cluster.first_seen = stamp;
            cluster.last_seen = stamp;
            cluster.observations = 1;
            cluster.mean_deviation = 0.0;
            clusters_.push_back(cluster);
            stats_.groups_passed++;
            for (size_t index : indices) output.keypoints.push_back(model_results.keypoints[index]);
            continue;
        }

        // Decide against the cluster as it stood before this observation, then fold the
        // observation in. Judging after the update would let a robot's own first frames inside a
        // static cluster tip the verdict on the same frame they arrive.
        const bool suppress = matched->is_static(config_, stamp);
        matched->observe(position, config_, stamp);

        if (suppress) {
            stats_.suppressed++;
            DiagnosticsData suppression;
            suppression["label"] = enum_to_string_lower(group_key.first);
            suppression["detection_index"] = group_key.second;
            suppression["field_x"] = position.x();
            suppression["field_y"] = position.y();
            suppression["dwell_s"] = stamp - matched->first_seen;
            suppression["observations"] = matched->observations;
            diagnostics_logger_->debug("suppressed", suppression);
            continue;
        }

        stats_.groups_passed++;
        for (size_t index : indices) output.keypoints.push_back(model_results.keypoints[index]);
    }

    stats_.tracked_clusters = static_cast<int>(clusters_.size());
    stats_.static_clusters = static_cast<int>(
        std::count_if(clusters_.begin(), clusters_.end(),
                      [&](const Cluster &cluster) { return cluster.is_static(config_, stamp); }));

    DiagnosticsData summary;
    summary["groups_in"] = stats_.groups_in;
    summary["groups_passed"] = stats_.groups_passed;
    summary["suppressed"] = stats_.suppressed;
    summary["tracked_clusters"] = stats_.tracked_clusters;
    summary["static_clusters"] = stats_.static_clusters;
    diagnostics_logger_->debug(summary);

    return output;
}

}  // namespace auto_battlebot
