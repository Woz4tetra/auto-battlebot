#include "robot_filter/ground_truth_robot_filter.hpp"

#include <spdlog/spdlog.h>

#include <chrono>

#include "transform_utils.hpp"

namespace auto_battlebot {

bool GroundTruthRobotFilter::initialize(int opponent_count) {
    if (opponent_count < 1 || opponent_count > 3) return false;

    our_frame_ids_.clear();
    opponent_frame_ids_.clear();
    connection_ = SimConnection::instance();

    static const FrameId our_ids[] = {FrameId::OUR_ROBOT_1};
    static const FrameId opp_ids[] = {FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2,
                                      FrameId::THEIR_ROBOT_3};

    our_frame_ids_.push_back(our_ids[0]);
    for (int i = 0; i < opponent_count; ++i) {
        opponent_frame_ids_.push_back(opp_ids[i]);
    }
    return true;
}

RobotDescriptionsStamped GroundTruthRobotFilter::update(
    [[maybe_unused]] KeypointsStamped keypoints, [[maybe_unused]] FieldDescription field,
    [[maybe_unused]] CameraInfo camera_info, [[maybe_unused]] KeypointsStamped robot_blob_keypoints,
    [[maybe_unused]] CommandFeedback cf) {
    RobotDescriptionsStamped result;
    result.header.frame_id = FrameId::FIELD;
    result.header.stamp =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

    const auto &gt = connection_->last_ground_truth_poses();
    if (gt.empty()) return result;

    // GT order: [0] = our robot, [1:] = opponents (matches sim config order)
    size_t gt_idx = 0;
    for (size_t i = 0; i < our_frame_ids_.size() && gt_idx < gt.size(); ++i, ++gt_idx) {
        RobotDescription desc;
        desc.frame_id = our_frame_ids_[i];
        desc.label = Label::EMPTY;
        desc.group = Group::OURS;
        desc.pose = pose2d_to_pose(gt[gt_idx]);
        desc.size = Size{0.15, 0.15, 0.1};
        desc.is_stale = false;
        result.descriptions.push_back(desc);
    }

    for (size_t i = 0; i < opponent_frame_ids_.size() && gt_idx < gt.size(); ++i, ++gt_idx) {
        RobotDescription desc;
        desc.frame_id = opponent_frame_ids_[i];
        desc.label = Label::OPPONENT;
        desc.group = Group::THEIRS;
        desc.pose = pose2d_to_pose(gt[gt_idx]);
        desc.size = Size{0.15, 0.15, 0.1};
        desc.is_stale = false;
        result.descriptions.push_back(desc);
    }

    return result;
}

}  // namespace auto_battlebot
