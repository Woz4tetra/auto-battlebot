#include "robot_filter/robot_front_back_filter.hpp"

#include <algorithm>
#include <cmath>
#include <magic_enum.hpp>
#include <numeric>
#include <set>

#include "data_structures/robot.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "enums/frame_id.hpp"
#include "robot_filter/label_group_utils.hpp"
#include "transform_utils.hpp"

namespace {
double position_distance(const auto_battlebot::Position &a, const auto_battlebot::Position &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool is_within_field_bounds(const auto_battlebot::Position &position,
                            const auto_battlebot::FieldDescription &field, double margin_meters) {
    const double width = field.size.size.x;
    const double height = field.size.size.y;
    if (width <= 0.0 || height <= 0.0) return true;
    const double half_x = width * 0.5 + margin_meters;
    const double half_y = height * 0.5 + margin_meters;
    return std::abs(position.x) <= half_x && std::abs(position.y) <= half_y;
}

const auto_battlebot::Position &position_of(const auto_battlebot::RobotDescription &d) {
    return d.pose.position;
}
const auto_battlebot::Position &position_of(const auto_battlebot::RobotKeypointDetection &d) {
    return d.description.pose.position;
}

template <typename T>
void erase_out_of_field(std::vector<T> &items, const auto_battlebot::FieldDescription &field,
                        double margin_meters) {
    items.erase(std::remove_if(items.begin(), items.end(),
                               [&field, margin_meters](const T &item) {
                                   return !is_within_field_bounds(position_of(item), field,
                                                                  margin_meters);
                               }),
                items.end());
}

auto_battlebot::Group infer_group_from_frame_ids(
    const std::vector<auto_battlebot::FrameId> &frame_ids) {
    for (const auto frame_id : frame_ids) {
        switch (frame_id) {
            case auto_battlebot::FrameId::OUR_ROBOT_1:
            case auto_battlebot::FrameId::OUR_ROBOT_2:
                return auto_battlebot::Group::OURS;
            case auto_battlebot::FrameId::NEUTRAL_ROBOT_1:
            case auto_battlebot::FrameId::NEUTRAL_ROBOT_2:
                return auto_battlebot::Group::NEUTRAL;
            case auto_battlebot::FrameId::THEIR_ROBOT_1:
            case auto_battlebot::FrameId::THEIR_ROBOT_2:
            case auto_battlebot::FrameId::THEIR_ROBOT_3:
                return auto_battlebot::Group::THEIRS;
            default:
                break;
        }
    }
    return auto_battlebot::Group::THEIRS;
}

}  // namespace

namespace auto_battlebot {
RobotFrontBackFilter::RobotFrontBackFilter(RobotFrontBackFilterConfiguration &config)
    : diagnostics_logger_(DiagnosticsLogger::get_logger("robot_front_back_filter")),
      label_to_frame_ids_(config.label_to_frame_ids),
      default_frame_id_(config.default_frame_id),
      max_jump_distance_(config.max_jump_distance),
      max_consecutive_jump_rejects_(config.max_consecutive_jump_rejects),
      blob_overwrite_min_distance_meters_(config.blob_overwrite_min_distance_meters),
      blob_overwrite_size_scale_(config.blob_overwrite_size_scale),
      size_overrides_(config.robot_size_meters_per_label),
      field_bounds_margin_meters_(config.field_bounds_margin_meters),
      our_robot_hold_window_s_(config.our_robot_hold_window_s),
      our_keypoint_dropout_blob_radius_meters_(config.our_keypoint_dropout_blob_radius_meters),
      our_keypoint_dropout_blob_window_s_(config.our_keypoint_dropout_blob_window_s),
      robot_keypoint_tracker_(config.robot_keypoint_tracker_config),
      frame_id_assigner_(config.max_jump_distance, config.max_consecutive_jump_rejects),
      motion_estimator_(make_motion_estimator(*config.motion_estimator)) {
    FrontBackKeypointConverterConfig converter_config;
    converter_config.front_keypoints = config.front_keypoints;
    converter_config.back_keypoints = config.back_keypoints;
    converter_config.keypoint_heights = config.robot_keypoint_tracker_config.keypoint_heights;
    keypoint_converter_ = std::make_unique<FrontBackKeypointConverter>(converter_config);
}

bool RobotFrontBackFilter::initialize(int opponent_count) {
    if (opponent_count < 1 || opponent_count > 3) {
        spdlog::error("Invalid opponent count: {}", opponent_count);
        return false;
    }

    static const std::vector<FrameId> opponent_frame_ids = {
        FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2, FrameId::THEIR_ROBOT_3};
    std::vector<FrameId> opponent_mapping;
    for (int i = 0; i < opponent_count; ++i) {
        opponent_mapping.push_back(opponent_frame_ids[static_cast<size_t>(i)]);
    }
    label_to_frame_ids_[Label::OPPONENT] = opponent_mapping;

    robot_configs_.clear();
    frame_id_assigner_.reset();
    motion_estimator_->reset();
    our_blob_present_no_keypoint_ = false;
    has_last_our_position_ = false;
    last_our_size_x_ = 0.0;
    last_our_position_stamp_ = 0.0;
    for (const auto &[label, frame_ids] : label_to_frame_ids_) {
        robot_configs_[label] = RobotConfig{label, infer_group_from_frame_ids(frame_ids)};
    }
    robot_configs_[Label::OPPONENT] = RobotConfig{Label::OPPONENT, Group::THEIRS};
    robot_keypoint_tracker_.set_robot_configs(robot_configs_);
    return true;
}

void RobotFrontBackFilter::predict(double now, CommandFeedback command_feedback) {
    motion_estimator_->predict(now, command_feedback);
    if (auto coasted = motion_estimator_->coast(now)) {
        state_.header.frame_id = FrameId::FIELD;
        state_.header.stamp = now;
        state_.descriptions = std::move(*coasted);
        apply_size_overrides(state_.descriptions);
    }
}

void RobotFrontBackFilter::correct(ModelResultStamped keypoints, FieldDescription field,
                                   CameraInfo camera_info,
                                   ModelResultStamped robot_blob_keypoints) {
    RobotDescriptionsStamped result;
    result.header.frame_id = FrameId::FIELD;
    result.header.stamp = keypoints.header.stamp;
    const Eigen::Matrix4d tf_fieldcenter_from_camera =
        field.tf_camera_from_fieldcenter.tf.inverse();

    our_blob_present_no_keypoint_ = false;

    auto all_measurements = convert_keypoints_to_measurements(keypoints, field, camera_info,
                                                              tf_fieldcenter_from_camera);
    erase_out_of_field(all_measurements, field, field_bounds_margin_meters_);
    const std::vector<RobotDescription> keypoint_measurements = all_measurements;
    const int num_keypoint_measurements = static_cast<int>(all_measurements.size());

    if (!robot_blob_keypoints.keypoints.empty() && field.child_frame_id != FrameId::EMPTY) {
        merge_blob_detections(robot_blob_keypoints, keypoint_measurements, field, camera_info,
                              tf_fieldcenter_from_camera, result.header.stamp, all_measurements);
    }

    const int num_measurements_before_temporal = static_cast<int>(all_measurements.size());
    MotionEstimatorContext estimator_context;
    estimator_context.tf_fieldcenter_from_camera = tf_fieldcenter_from_camera;
    estimator_context.camera_info = camera_info;
    estimator_context.field_bounds_margin_meters = field_bounds_margin_meters_;
    estimator_context.our_robot_hold_window_s = our_robot_hold_window_s_;
    result.descriptions =
        motion_estimator_->update(std::move(all_measurements), result.header.stamp,
                                  frame_id_assigner_, field, estimator_context);

    // Anchor the held OUR_ROBOT_1 pose from this frame's output (measured or predicted) so the next
    // frame's blob suppression has a reference even when our keypoint drops out.
    update_our_position_anchor(result.descriptions, result.header.stamp);

    diagnostics_logger_->debug(
        {{"num_input_keypoints", static_cast<int>(keypoints.keypoints.size())},
         {"num_input_blob_keypoints", static_cast<int>(robot_blob_keypoints.keypoints.size())},
         {"field_frame_valid", field.child_frame_id == FrameId::EMPTY ? "false" : "true"},
         {"num_keypoint_measurements", num_keypoint_measurements},
         {"num_measurements_before_temporal", num_measurements_before_temporal},
         {"num_measurements_after_temporal", static_cast<int>(result.descriptions.size())}});

    state_ = std::move(result);
    apply_size_overrides(state_.descriptions);
}

void RobotFrontBackFilter::apply_size_overrides(std::vector<RobotDescription> &descriptions) const {
    // Replace the detector's box with the robot's known dimensions, once, here: every consumer
    // downstream reads `size`, so overriding it is what keeps the keep-out, the drawn circle and
    // the marker cube from disagreeing about how big a robot is.
    //
    // The measured box is logged rather than discarded. It is the only signal that the detector
    // has started boxing a robot badly, and overwriting it without a trace would hide exactly the
    // regression these overrides exist to compensate for.
    DiagnosticsData detected;
    for (auto &description : descriptions) {
        const auto found = size_overrides_.find(description.label);
        if (found == size_overrides_.end()) continue;
        const std::string label(magic_enum::enum_name(description.label));
        detected[label + "_x"] = description.size.x;
        detected[label + "_y"] = description.size.y;
        detected[label + "_z"] = description.size.z;
        description.size = found->second;
    }
    if (!detected.empty()) diagnostics_logger_->debug("detected_size", detected);
}

bool RobotFrontBackFilter::is_blob_suppressed_by_keypoint(
    const RobotKeypointDetection &blob,
    const std::vector<RobotDescription> &keypoint_measurements) const {
    // Keypoint detections are higher quality than blob detections. When a blob overlaps a
    // keypoint measurement we drop the blob to avoid double-counting the same robot.
    //
    // The suppression radius adapts to the sizes of both detections so that large robots have
    // a larger exclusion zone. A minimum distance prevents suppression from collapsing to zero
    // when both size estimates are small or zero.
    //
    //   radius = max(min_distance, size_scale * (blob.size.x + keypoint.size.x))
    return std::any_of(keypoint_measurements.begin(), keypoint_measurements.end(),
                       [this, &blob](const RobotDescription &keypoint_measurement) {
                           const double overwrite_radius =
                               std::max(blob_overwrite_min_distance_meters_,
                                        blob_overwrite_size_scale_ * (blob.description.size.x +
                                                                      keypoint_measurement.size.x));
                           const double dist = position_distance(
                               blob.description.pose.position, keypoint_measurement.pose.position);
                           return dist <= overwrite_radius;
                       });
}

bool RobotFrontBackFilter::is_our_anchor_fresh(double stamp) const {
    if (!has_last_our_position_) return false;
    if (our_keypoint_dropout_blob_window_s_ <= 0.0) return false;
    const double age = stamp - last_our_position_stamp_;
    return age >= 0.0 && age <= our_keypoint_dropout_blob_window_s_;
}

int RobotFrontBackFilter::suppress_blobs_near_our_anchor(
    double stamp, const std::vector<RobotDescription> &keypoint_measurements,
    std::vector<RobotKeypointDetection> &blobs) const {
    if (our_keypoint_dropout_blob_radius_meters_ <= 0.0) return 0;
    // Needs a recent held pose for our robot; without one there is nothing to suppress against.
    if (!is_our_anchor_fresh(stamp)) return 0;

    // Only a tick where our robot's keypoint is missing can leak. When the keypoint is present,
    // is_blob_suppressed_by_keypoint has already handled any blob sitting on top of it.
    const bool our_keypoint_present =
        std::any_of(keypoint_measurements.begin(), keypoint_measurements.end(),
                    [](const RobotDescription &measurement) {
                        return measurement.frame_id == FrameId::OUR_ROBOT_1;
                    });
    if (our_keypoint_present) return 0;

    // The blob model has no class for our robot, so a blob here would be assigned an opponent
    // FrameId at our own position. Drop it rather than let target selection steer into us.
    const auto removed_begin =
        std::remove_if(blobs.begin(), blobs.end(), [this](const RobotKeypointDetection &blob) {
            return position_distance(blob.description.pose.position, last_our_position_) <=
                   our_keypoint_dropout_blob_radius_meters_;
        });
    const int num_removed = static_cast<int>(std::distance(removed_begin, blobs.end()));
    blobs.erase(removed_begin, blobs.end());
    return num_removed;
}

void RobotFrontBackFilter::update_our_position_anchor(
    const std::vector<RobotDescription> &descriptions, double stamp) {
    for (const auto &description : descriptions) {
        if (description.frame_id == FrameId::OUR_ROBOT_1) {
            has_last_our_position_ = true;
            last_our_position_ = description.pose.position;
            last_our_size_x_ = description.size.x;
            last_our_position_stamp_ = stamp;
            return;
        }
    }
    // Our robot is no longer tracked, but the anchor is kept on purpose: our_robot_hold_window_s_
    // drops the track well before a typical keypoint dropout ends, and suppression still needs a
    // position to work from. is_our_anchor_fresh() ages it out instead, freezing the zone at the
    // last held pose rather than clearing it here.
}

std::vector<FrameId> RobotFrontBackFilter::get_assignment_frame_ids(
    Label label, const std::vector<RobotDescription> &used_measurements) const {
    auto is_used = [&used_measurements](FrameId fid) {
        return std::any_of(used_measurements.begin(), used_measurements.end(),
                           [fid](const RobotDescription &m) { return m.frame_id == fid; });
    };

    std::vector<FrameId> assignment_fids;
    for (const auto &fid : get_frame_ids_for_label(label)) {
        if (!is_used(fid)) assignment_fids.push_back(fid);
    }

    // If a specific THEIRS label has no free ids (e.g. mapped only to THEIR_ROBOT_1),
    // allow remaining blob detections to use free OPPONENT ids.
    if (assignment_fids.empty() && label != Label::OPPONENT &&
        group_for_label(label, robot_configs_) == Group::THEIRS) {
        for (const auto &fid : get_frame_ids_for_label(Label::OPPONENT)) {
            if (!is_used(fid)) assignment_fids.push_back(fid);
        }
    }

    return assignment_fids;
}

void RobotFrontBackFilter::merge_blob_detections(
    const ModelResultStamped &robot_blob_keypoints,
    const std::vector<RobotDescription> &keypoint_measurements, const FieldDescription &field,
    const CameraInfo &camera_info, const Eigen::Matrix4d &tf_fieldcenter_from_camera, double stamp,
    std::vector<RobotDescription> &all_measurements) {
    auto blob_detections =
        robot_keypoint_tracker_.detect_with_confidence(robot_blob_keypoints, field, camera_info);
    const int blob_candidates_before_overwrite = static_cast<int>(blob_detections.size());

    // Convert blob detections from camera to map
    for (auto &blob_detection : blob_detections) {
        auto blob_pos = blob_detection.description.pose.position;
        const Eigen::Vector3d blob_center_camera(blob_pos.x, blob_pos.y, blob_pos.z);
        blob_detection.description.pose.position =
            vector_to_position(transform_point(tf_fieldcenter_from_camera, blob_center_camera));
    }

    erase_out_of_field(blob_detections, field, field_bounds_margin_meters_);

    blob_detections.erase(
        std::remove_if(blob_detections.begin(), blob_detections.end(),
                       [this, &keypoint_measurements](const RobotKeypointDetection &blob) {
                           return is_blob_suppressed_by_keypoint(blob, keypoint_measurements);
                       }),
        blob_detections.end());

    // Our robot is invisible to the blob model, so while its keypoints are missing any blob on
    // our held pose is almost certainly us. Drop those before FrameId assignment can hand them
    // an opponent slot at our own position.
    const int num_blobs_suppressed_near_us =
        suppress_blobs_near_our_anchor(stamp, keypoint_measurements, blob_detections);
    our_blob_present_no_keypoint_ = num_blobs_suppressed_near_us > 0;

    diagnostics_logger_->debug(
        {{"num_blob_candidates_before_overwrite", blob_candidates_before_overwrite},
         {"num_blob_candidates_after_overwrite", static_cast<int>(blob_detections.size())},
         {"num_blob_candidates_suppressed",
          blob_candidates_before_overwrite - static_cast<int>(blob_detections.size())},
         {"num_blob_candidates_suppressed_near_us", num_blobs_suppressed_near_us}});

    std::map<Label, std::vector<MeasurementWithConfidence>> grouped_blob_measurements;
    for (auto &blob_detection : blob_detections) {
        grouped_blob_measurements[blob_detection.description.label].push_back(
            {blob_detection.confidence, std::move(blob_detection.description)});
    }
    diagnostics_logger_->debug(
        {{"num_blob_labels_with_candidates", static_cast<int>(grouped_blob_measurements.size())}});

    // Pool every blob measurement into a single global assignment. For each measurement we
    // record its allowed FrameIds (its label's own configured slots, with THEIRS-fallback to
    // OPPONENT slots), then call the assigner once. Iteration order across labels no longer
    // affects the result -- a previous per-label loop let label enum order decide which label
    // got first claim on shared OPPONENT slots, silently dropping detections whose label
    // happened to sort last.
    std::vector<MeasurementWithConfidence> pool;
    std::vector<std::vector<FrameId>> pool_allowed_fids;
    std::set<FrameId> available_fids_set;
    for (auto &[label, measurements] : grouped_blob_measurements) {
        const std::vector<FrameId> allowed = get_assignment_frame_ids(label, keypoint_measurements);
        for (auto fid : allowed) available_fids_set.insert(fid);
        diagnostics_logger_->debug(
            std::string("blob_pool_") + std::string(magic_enum::enum_name(label)),
            {{"num_candidates", static_cast<int>(measurements.size())},
             {"num_allowed_fids", static_cast<int>(allowed.size())}});
        for (auto &m : measurements) {
            pool_allowed_fids.push_back(allowed);
            pool.push_back(std::move(m));
        }
    }

    if (pool.empty() || available_fids_set.empty()) {
        diagnostics_logger_->debug(
            {{"num_blob_pool", static_cast<int>(pool.size())},
             {"num_available_fids", static_cast<int>(available_fids_set.size())},
             {"num_assigned", 0}});
        return;
    }

    // Sort by confidence so higher-confidence blobs get priority on distance ties in the
    // global greedy assignment. We sort indices to keep `pool_allowed_fids` aligned.
    std::vector<size_t> order(pool.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&pool](size_t a, size_t b) { return pool[a].confidence > pool[b].confidence; });
    std::vector<MeasurementWithConfidence> sorted_pool;
    std::vector<std::vector<FrameId>> sorted_allowed;
    sorted_pool.reserve(pool.size());
    sorted_allowed.reserve(pool.size());
    for (size_t i : order) {
        sorted_pool.push_back(std::move(pool[i]));
        sorted_allowed.push_back(std::move(pool_allowed_fids[i]));
    }

    const std::vector<FrameId> available_fids(available_fids_set.begin(), available_fids_set.end());
    auto assigned =
        frame_id_assigner_.assign(sorted_pool, available_fids, diagnostics_logger_, sorted_allowed);

    diagnostics_logger_->debug({{"num_blob_pool", static_cast<int>(sorted_pool.size())},
                                {"num_available_fids", static_cast<int>(available_fids.size())},
                                {"num_assigned", static_cast<int>(assigned.size())}});

    for (auto &a : assigned) {
        // Orientation from blob rectangles is intentionally non-semantic. Keep identity.
        a.pose.rotation = Rotation{1.0, 0.0, 0.0, 0.0};
        // If this measurement got a FrameId outside its label's own configured slots, the
        // THEIRS-fallback to OPPONENT slots fired. Re-label to OPPONENT so (label, frame_id,
        // group) stay consistent -- otherwise downstream consumers that key off `label` see a
        // specific-model description sitting in a generic opponent slot, and the label flips
        // back to its specific value the next tick a "proper" slot frees up (an ID swap).
        const std::vector<FrameId> own_fids = get_frame_ids_for_label(a.label);
        const bool used_fallback =
            std::find(own_fids.begin(), own_fids.end(), a.frame_id) == own_fids.end();
        if (used_fallback) a.label = Label::OPPONENT;
        a.group = group_for_frame_id(a.frame_id, a.group);
        all_measurements.push_back(std::move(a));
    }
}

std::vector<RobotDescription> RobotFrontBackFilter::convert_keypoints_to_measurements(
    const ModelResultStamped &model_results, const FieldDescription &field,
    const CameraInfo &camera_info, const Eigen::Matrix4d &tf_fieldcenter_from_camera) {
    std::vector<RobotDescription> filter_measurements;
    auto front_back_mapping = keypoint_converter_->convert(model_results, field, camera_info);
    diagnostics_logger_->debug(
        {{"num_labels_with_front_back_assignments", static_cast<int>(front_back_mapping.size())}});

    // Clear stale last_position for single-FrameId labels that have no detections this frame,
    // so when they reappear we assign without bias from an old position.
    int cleared_single_label_positions = 0;
    for (const auto &[label, frame_ids] : label_to_frame_ids_) {
        if (frame_ids.size() != 1) continue;
        if (front_back_mapping.count(label) == 0) {
            frame_id_assigner_.clear_last_position(frame_ids[0]);
            ++cleared_single_label_positions;
        }
    }
    diagnostics_logger_->debug(
        {{"num_cleared_single_label_last_positions", cleared_single_label_positions}});

    for (const auto &[label, assignments_with_conf] : front_back_mapping) {
        std::vector<MeasurementWithConfidence> valid_measurements =
            build_valid_measurements(tf_fieldcenter_from_camera, label, assignments_with_conf);
        std::vector<FrameId> frame_ids = get_frame_ids_for_label(label);
        const std::string label_context =
            std::string("keypoint_assignment_") + std::string(magic_enum::enum_name(label));

        if (valid_measurements.empty()) {
            if (frame_ids.size() == 1) frame_id_assigner_.clear_last_position(frame_ids[0]);
            diagnostics_logger_->debug(
                label_context,
                {{"num_front_back_assignments", static_cast<int>(assignments_with_conf.size())},
                 {"num_valid_measurements", static_cast<int>(valid_measurements.size())},
                 {"num_frame_ids", static_cast<int>(frame_ids.size())},
                 {"num_assigned", 0},
                 {"cleared_last_position_due_to_empty", frame_ids.size() == 1 ? "true" : "false"}});
            continue;
        }

        std::sort(valid_measurements.begin(), valid_measurements.end(),
                  [](const MeasurementWithConfidence &a, const MeasurementWithConfidence &b) {
                      return a.confidence > b.confidence;
                  });

        std::vector<RobotDescription> assigned =
            frame_id_assigner_.assign(valid_measurements, frame_ids, diagnostics_logger_);
        diagnostics_logger_->debug(
            label_context,
            {{"num_front_back_assignments", static_cast<int>(assignments_with_conf.size())},
             {"num_valid_measurements", static_cast<int>(valid_measurements.size())},
             {"num_frame_ids", static_cast<int>(frame_ids.size())},
             {"num_assigned", static_cast<int>(assigned.size())},
             {"cleared_last_position_due_to_empty", "false"}});
        for (auto &desc : assigned) {
            desc.group = group_for_frame_id(desc.frame_id, desc.group);
            filter_measurements.push_back(std::move(desc));
        }
    }
    diagnostics_logger_->debug(
        {{"num_keypoint_measurements_assigned", static_cast<int>(filter_measurements.size())}});
    return filter_measurements;
}

std::vector<MeasurementWithConfidence> RobotFrontBackFilter::build_valid_measurements(
    const Eigen::Matrix4d &tf_fieldcenter_from_camera, Label label,
    const std::vector<std::pair<FrontBackAssignment, double>> &assignments_with_conf) {
    std::vector<MeasurementWithConfidence> valid_measurements;
    std::string label_str = std::string(magic_enum::enum_name(label));
    const Group group = group_for_label(label, robot_configs_);

    for (const auto &[assignment, confidence] : assignments_with_conf) {
        Eigen::Vector3d front_keypoint_in_field =
            transform_point(tf_fieldcenter_from_camera, assignment.front);
        Eigen::Vector3d back_keypoint_in_field =
            transform_point(tf_fieldcenter_from_camera, assignment.back);
        double length = (front_keypoint_in_field - back_keypoint_in_field).norm();

        Transform tf_field_from_robot;
        bool pose_found = keypoint_converter_->get_pose_from_points(
            front_keypoint_in_field, back_keypoint_in_field, tf_field_from_robot);
        diagnostics_logger_->debug(label_str, {{"pose_found", pose_found ? "true" : "false"}});

        if (!pose_found) continue;

        Pose pose = matrix_to_pose(tf_field_from_robot.tf);
        RobotDescription desc{FrameId::EMPTY,
                              label,
                              group,
                              pose,
                              Size{length, length, 0.1},
                              {vector_to_position(front_keypoint_in_field),
                               vector_to_position(back_keypoint_in_field)},
                              Velocity2D{},
                              false};
        valid_measurements.push_back({confidence, std::move(desc)});
    }
    return valid_measurements;
}

std::vector<FrameId> RobotFrontBackFilter::get_frame_ids_for_label(Label label) const {
    if (label_to_frame_ids_.count(label) != 0 && !label_to_frame_ids_.at(label).empty())
        return label_to_frame_ids_.at(label);
    return {get_default_frame_id_for_label(label)};
}

FrameId RobotFrontBackFilter::get_default_frame_id_for_label(const Label label) const {
    if (label_to_frame_ids_.count(label) == 0) return default_frame_id_;
    const std::vector<FrameId> &v = label_to_frame_ids_.at(label);
    if (v.empty()) return default_frame_id_;
    return v[0];
}

}  // namespace auto_battlebot
