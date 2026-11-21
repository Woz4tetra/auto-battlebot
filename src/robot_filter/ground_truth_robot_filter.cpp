#include "robot_filter/ground_truth_robot_filter.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {
constexpr double kMinUsableIntervalS = 1e-3;
constexpr double kMaxUsableIntervalS = 0.5;
}  // namespace

bool GroundTruthRobotFilter::initialize(int opponent_count) {
    if (opponent_count < 1 || opponent_count > 3) return false;

    our_frame_ids_.clear();
    opponent_frame_ids_.clear();
    neutral_frame_ids_.clear();
    connection_ = SimConnection::instance();

    static const FrameId our_ids[] = {FrameId::OUR_ROBOT_1};
    static const FrameId opp_ids[] = {FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2,
                                      FrameId::THEIR_ROBOT_3};
    static const FrameId neutral_ids[] = {FrameId::NEUTRAL_ROBOT_1, FrameId::NEUTRAL_ROBOT_2};

    our_frame_ids_.push_back(our_ids[0]);
    for (int i = 0; i < opponent_count; ++i) {
        opponent_frame_ids_.push_back(opp_ids[i]);
    }
    const int neutrals = std::clamp(trailing_neutral_slot_count_, 0, 2);
    for (int i = 0; i < neutrals; ++i) {
        neutral_frame_ids_.push_back(neutral_ids[i]);
    }
    return true;
}

void GroundTruthRobotFilter::correct([[maybe_unused]] ModelResultStamped keypoints,
                                     [[maybe_unused]] FieldDescription field,
                                     [[maybe_unused]] CameraInfo camera_info) {
    RobotDescriptionsStamped result;
    result.header.frame_id = FrameId::FIELD;
    result.header.stamp = clock_->now();

    const SimMeasurementFrame &frame = connection_->last_measurements();
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    std::vector<Pose2D> gt;
    gt.reserve(frame.robots.size());
    for (const SimRobotMeasurement &robot : frame.robots) {
        gt.push_back(frame.delivered && robot.measured ? robot.pose : Pose2D{kNaN, kNaN, kNaN});
    }
    if (gt.empty()) {
        state_ = result;
        return;
    }

    const double now = result.header.stamp;
    const double dt = prev_stamp_ > 0.0 ? now - prev_stamp_ : 0.0;

    size_t gt_idx = 0;
    for (size_t i = 0; i < our_frame_ids_.size() && gt_idx < gt.size(); ++i, ++gt_idx) {
        auto desc = describe(our_frame_ids_[i], Label::EMPTY, Group::OURS, gt[gt_idx], now, dt);
        if (desc) result.descriptions.push_back(*desc);
    }

    for (size_t i = 0; i < opponent_frame_ids_.size() && gt_idx < gt.size(); ++i, ++gt_idx) {
        auto desc =
            describe(opponent_frame_ids_[i], Label::OPPONENT, Group::THEIRS, gt[gt_idx], now, dt);
        if (desc) result.descriptions.push_back(*desc);
    }

    for (size_t i = 0; i < neutral_frame_ids_.size() && gt_idx < gt.size(); ++i, ++gt_idx) {
        auto desc =
            describe(neutral_frame_ids_[i], Label::HOUSE_BOT, Group::NEUTRAL, gt[gt_idx], now, dt);
        if (desc) result.descriptions.push_back(*desc);
    }

    prev_stamp_ = result.header.stamp;
    state_ = result;
}

std::optional<RobotDescription> GroundTruthRobotFilter::describe(FrameId frame_id, Label label,
                                                                 Group group, const Pose2D &gt_pose,
                                                                 double now, double dt) {
    RobotDescription desc;
    desc.frame_id = frame_id;
    desc.label = label;
    desc.group = group;
    desc.size = Size{0.15, 0.15, 0.1};

    if (!std::isfinite(gt_pose.x) || !std::isfinite(gt_pose.y) || !std::isfinite(gt_pose.yaw)) {
        const auto held = prev_poses_.find(frame_id);
        if (held == prev_poses_.end()) return std::nullopt;

        desc.is_stale = true;
        const auto velocity = prev_velocities_.find(frame_id);
        Pose2D coasted = held->second;
        if (velocity != prev_velocities_.end() && interval_usable(dt)) {
            desc.velocity = velocity->second;
            coasted.x += desc.velocity.vx_mps * dt;
            coasted.y += desc.velocity.vy_mps * dt;
            coasted.yaw = normalize_angle(coasted.yaw + desc.velocity.omega_radps * dt);
        } else if (velocity != prev_velocities_.end()) {
            desc.velocity = velocity->second;
        }
        prev_poses_[frame_id] = coasted;
        desc.pose = pose2d_to_pose(coasted);
        return desc;
    }

    desc.pose = pose2d_to_pose(gt_pose);
    desc.is_stale = false;
    desc.velocity = velocity_from_delta(frame_id, gt_pose, now);
    prev_velocities_[frame_id] = desc.velocity;
    return desc;
}

bool GroundTruthRobotFilter::interval_usable(double interval) {
    return interval > kMinUsableIntervalS && interval < kMaxUsableIntervalS;
}

double GroundTruthRobotFilter::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Velocity2D GroundTruthRobotFilter::velocity_from_delta(FrameId frame_id, const Pose2D &pose,
                                                       double now) {
    const auto found = measured_poses_.find(frame_id);
    const auto stamped = measured_stamps_.find(frame_id);
    const bool have_previous = found != measured_poses_.end() && stamped != measured_stamps_.end();
    const Pose2D previous = have_previous ? found->second : pose;
    const double elapsed = have_previous ? now - stamped->second : 0.0;

    prev_poses_[frame_id] = pose;
    measured_poses_[frame_id] = pose;
    measured_stamps_[frame_id] = now;

    if (!have_previous || !interval_usable(elapsed)) return Velocity2D{};
    return Velocity2D{
        (pose.x - previous.x) / elapsed,
        (pose.y - previous.y) / elapsed,
        normalize_angle(pose.yaw - previous.yaw) / elapsed,
    };
}

}  // namespace auto_battlebot
