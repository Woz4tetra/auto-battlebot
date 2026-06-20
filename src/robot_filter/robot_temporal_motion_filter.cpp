#include "robot_filter/robot_temporal_motion_filter.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include "transform_utils.hpp"

namespace {
void clip_to_field_bounds(auto_battlebot::Position &position,
                          const auto_battlebot::FieldDescription &field, double margin_meters) {
    const double width = field.size.size.x;
    const double height = field.size.size.y;
    if (width <= 0.0 || height <= 0.0) return;
    const double half_x = width * 0.5 + margin_meters;
    const double half_y = height * 0.5 + margin_meters;
    position.x = std::clamp(position.x, -half_x, half_x);
    position.y = std::clamp(position.y, -half_y, half_y);
}
}  // namespace

namespace auto_battlebot {
void RobotTemporalMotionFilter::reset() {
    last_timestamp_per_frame_id_.clear();
    last_description_per_frame_id_.clear();
}

std::vector<RobotDescription> RobotTemporalMotionFilter::update_with_prediction(
    std::vector<RobotDescription> inputs, const CommandFeedback &command_feedback, double timestamp,
    FrameIdAssigner &frame_id_assigner, const FieldDescription &field,
    double field_bounds_margin_meters) {
    std::set<FrameId> measured_frame_ids;

    for (const auto &input : inputs) {
        if (input.frame_id == FrameId::EMPTY) continue;
        measured_frame_ids.insert(input.frame_id);
        last_description_per_frame_id_[input.frame_id] = input;
    }

    for (auto &[frame_id, desc] : last_description_per_frame_id_) {
        if (measured_frame_ids.count(frame_id) != 0) continue;

        auto cmd_it = command_feedback.commands.find(frame_id);
        if (cmd_it == command_feedback.commands.end()) {
            // Opponents (or ours without feedback): hold last known pose.
            continue;
        }

        auto ts_it = last_timestamp_per_frame_id_.find(frame_id);
        if (ts_it == last_timestamp_per_frame_id_.end()) continue;

        const double dt = timestamp - ts_it->second;
        if (dt <= 0.0 || dt > 1.0) continue;

        const VelocityCommand &cmd = cmd_it->second;
        Pose2D predicted_pose = pose_to_pose2d(desc.pose);
        const Eigen::Vector2d velocity_field =
            body_velocity_to_field(cmd.linear_x, cmd.linear_y, predicted_pose.yaw);

        predicted_pose.x += velocity_field.x() * dt;
        predicted_pose.y += velocity_field.y() * dt;
        predicted_pose.yaw = std::atan2(std::sin(predicted_pose.yaw + cmd.angular_z * dt),
                                        std::cos(predicted_pose.yaw + cmd.angular_z * dt));
        desc.pose = pose2d_to_pose(predicted_pose);
        clip_to_field_bounds(desc.pose.position, field, field_bounds_margin_meters);
        frame_id_assigner.set_last_position(frame_id, desc.pose.position);
    }

    std::vector<RobotDescription> outputs;
    outputs.reserve(last_description_per_frame_id_.size());
    for (const auto &[frame_id, desc] : last_description_per_frame_id_) {
        RobotDescription output = desc;
        output.is_stale = measured_frame_ids.count(frame_id) == 0;
        // Refresh the per-robot timestamp so the next frame's dead-reckoning dt spans one frame.
        last_timestamp_per_frame_id_[frame_id] = timestamp;
        outputs.push_back(std::move(output));
    }
    return outputs;
}
}  // namespace auto_battlebot
