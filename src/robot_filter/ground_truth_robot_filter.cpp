#include "robot_filter/ground_truth_robot_filter.hpp"

#include <spdlog/spdlog.h>

#include <chrono>

#include "transform_utils.hpp"

namespace auto_battlebot {

bool GroundTruthRobotFilter::initialize(const std::vector<RobotConfig> &robots) {
    robot_configs_ = robots;
    our_frame_ids_.clear();
    opponent_frame_ids_.clear();
    connection_ = SimConnection::instance();

    int our_idx = 0;
    int opp_idx = 0;
    static const FrameId our_ids[] = {FrameId::OUR_ROBOT_1, FrameId::OUR_ROBOT_2};
    static const FrameId opp_ids[] = {FrameId::THEIR_ROBOT_1, FrameId::THEIR_ROBOT_2,
                                      FrameId::THEIR_ROBOT_3};
    for (const auto &rc : robots) {
        if (rc.group == Group::OURS && our_idx < 2) {
            our_frame_ids_.push_back(our_ids[our_idx++]);
        } else if (rc.group == Group::THEIRS && opp_idx < 3) {
            opponent_frame_ids_.push_back(opp_ids[opp_idx++]);
        }
    }
    return true;
}

RobotDescriptionsStamped GroundTruthRobotFilter::update([[maybe_unused]] KeypointsStamped keypoints,
                                                        [[maybe_unused]] FieldDescription field,
                                                        [[maybe_unused]] CameraInfo camera_info,
                                                        [[maybe_unused]] MaskStamped robot_mask,
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
        desc.label = robot_configs_[i].label;
        desc.pose = pose2d_to_pose(gt[gt_idx]);
        desc.size = Size{0.15, 0.15, 0.1};
        result.descriptions.push_back(desc);
    }

    for (size_t i = 0; i < opponent_frame_ids_.size() && gt_idx < gt.size(); ++i, ++gt_idx) {
        RobotDescription desc;
        desc.frame_id = opponent_frame_ids_[i];
        desc.label = Label::OPPONENT;
        desc.pose = pose2d_to_pose(gt[gt_idx]);
        desc.size = Size{0.15, 0.15, 0.1};
        result.descriptions.push_back(desc);
    }

    return result;
}

}  // namespace auto_battlebot
