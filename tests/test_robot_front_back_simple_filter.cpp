#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core.hpp>

#include "robot_filter/robot_front_back_simple_filter.hpp"

namespace auto_battlebot {
namespace {
CameraInfo make_camera_info() {
    CameraInfo camera_info;
    camera_info.width = 640;
    camera_info.height = 480;
    camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
    camera_info.intrinsics.at<double>(0, 0) = 500.0;
    camera_info.intrinsics.at<double>(1, 1) = 500.0;
    camera_info.intrinsics.at<double>(0, 2) = 320.0;
    camera_info.intrinsics.at<double>(1, 2) = 240.0;
    return camera_info;
}

FieldDescription make_field() {
    FieldDescription field;
    field.child_frame_id = FrameId::FIELD;
    field.size.size.x = 2.0;
    field.size.size.y = 2.0;
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 2) = -1.0;
    tf(2, 3) = 3.0;
    field.tf_camera_from_fieldcenter.tf = tf;
    return field;
}

KeypointsStamped make_opponent_keypoints(double stamp, double front_x, double back_x) {
    KeypointsStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;
    keypoints.header.stamp = stamp;

    Keypoint front;
    front.label = Label::OPPONENT;
    front.keypoint_label = KeypointLabel::OPPONENT_FRONT;
    front.x = front_x;
    front.y = 220.0;
    front.confidence = 0.9;
    front.detection_index = 1;
    keypoints.keypoints.push_back(front);

    Keypoint back = front;
    back.keypoint_label = KeypointLabel::OPPONENT_BACK;
    back.x = back_x;
    back.confidence = 0.85;
    keypoints.keypoints.push_back(back);
    return keypoints;
}

void append_keypoint_pair(KeypointsStamped &keypoints, Label label, KeypointLabel front_kp_label,
                          KeypointLabel back_kp_label, int detection_index, double front_x,
                          double back_x, double y, double confidence) {
    Keypoint front;
    front.label = label;
    front.keypoint_label = front_kp_label;
    front.x = front_x;
    front.y = y;
    front.confidence = confidence;
    front.detection_index = detection_index;
    keypoints.keypoints.push_back(front);

    Keypoint back = front;
    back.keypoint_label = back_kp_label;
    back.x = back_x;
    back.confidence = confidence;
    keypoints.keypoints.push_back(back);
}
}  // namespace

// when a specific THEIRS-label blob can't be assigned to its own configured
// FrameId (because a keypoint detection took it) and falls back to a generic OPPONENT slot,
// the resulting RobotDescription must be re-labeled to Label::OPPONENT so (label, frame_id,
// group) stay consistent. Otherwise downstream consumers see a specific-model label sitting
// in a generic opponent slot, and the label flips back to its specific value the next tick a
// "proper" slot frees up (an ID swap).
TEST(RobotFrontBackSimpleFilterTest, BlobFallbackToOpponentRelabelsToOpponent) {
    RobotFrontBackSimpleFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT, KeypointLabel::MRS_BUFF_MK1_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK, KeypointLabel::MRS_BUFF_MK1_BACK};
    config.label_to_frame_ids = {{Label::MRS_BUFF_MK1, {FrameId::THEIR_ROBOT_1}}};
    config.default_frame_id = FrameId::THEIR_ROBOT_1;
    config.max_jump_distance = 1.0;
    config.max_consecutive_jump_rejects = 1;
    config.robot_keypoint_tracker_config.min_length_meters = 0.05;
    config.robot_keypoint_tracker_config.max_length_meters = 2.0;
    config.robot_keypoint_tracker_config.min_confidence = 0.2;
    config.robot_keypoint_tracker_config.max_candidates = 8;

    RobotFrontBackSimpleFilter filter(config);
    // 2 opponents -> Label::OPPONENT maps to {THEIR_ROBOT_1, THEIR_ROBOT_2}, so MRS_BUFF_MK1's
    // own slot {THEIR_ROBOT_1} can be exhausted by an OPPONENT keypoint while a free OPPONENT
    // fallback slot remains.
    ASSERT_TRUE(filter.initialize(2));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const CommandFeedback command_feedback;

    // OPPONENT keypoint claims THEIR_ROBOT_1 (the only slot MRS_BUFF_MK1 is configured for).
    const KeypointsStamped opponent_keypoints = make_opponent_keypoints(1.0, 280.0, 320.0);

    // MRS_BUFF_MK1 blob, spatially separated from the keypoint detection so it survives the
    // is_blob_suppressed_by_keypoint filter. It can only land in THEIR_ROBOT_2 via the
    // THEIRS-fallback to OPPONENT slots.
    KeypointsStamped blob_keypoints;
    blob_keypoints.header.frame_id = FrameId::CAMERA;
    blob_keypoints.header.stamp = 1.0;
    append_keypoint_pair(blob_keypoints, Label::MRS_BUFF_MK1, KeypointLabel::MRS_BUFF_MK1_FRONT,
                         KeypointLabel::MRS_BUFF_MK1_BACK, 1, 430.0, 470.0, 240.0, 0.8);

    const auto result =
        filter.update(opponent_keypoints, field, camera_info, blob_keypoints, command_feedback);
    ASSERT_EQ(result.descriptions.size(), 2u);

    const RobotDescription *kp_meas = nullptr;
    const RobotDescription *blob_meas = nullptr;
    for (const auto &desc : result.descriptions) {
        if (desc.frame_id == FrameId::THEIR_ROBOT_1) kp_meas = &desc;
        if (desc.frame_id == FrameId::THEIR_ROBOT_2) blob_meas = &desc;
    }
    ASSERT_NE(kp_meas, nullptr);
    ASSERT_NE(blob_meas, nullptr);

    // Keypoint detection: original OPPONENT label, kept as-is.
    EXPECT_EQ(kp_meas->label, Label::OPPONENT);
    EXPECT_EQ(kp_meas->group, Group::THEIRS);

    // Blob detection: fed in as MRS_BUFF_MK1, assigned to THEIR_ROBOT_2 via fallback. The 4b
    // fix re-labels it to OPPONENT so label and frame_id agree.
    EXPECT_EQ(blob_meas->label, Label::OPPONENT);
    EXPECT_EQ(blob_meas->group, Group::THEIRS);
}

// 4a regression: the per-label loop over std::map<Label, ...> previously let Label enum order
// decide which label got first claim on shared OPPONENT FrameId slots. With OPPONENT (enum 7)
// sorting before MRS_BUFF_MK3 (enum 8), two generic-OPPONENT blobs could grab THEIR_ROBOT_1
// and THEIR_ROBOT_2 first, leaving the more-specific MRS_BUFF_MK3 blob with nothing -- even
// though MRS_BUFF_MK3 is configured to own THEIR_ROBOT_2 outright. The new global assignment
// honors per-measurement allowed-FrameId constraints in a single call, so the specific-label
// blob keeps its slot regardless of enum order.
TEST(RobotFrontBackSimpleFilterTest, GlobalAssignmentDoesNotDropSpecificLabelBlob) {
    RobotFrontBackSimpleFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT, KeypointLabel::MRS_BUFF_MK3_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK, KeypointLabel::MRS_BUFF_MK3_BACK};
    config.label_to_frame_ids = {{Label::MRS_BUFF_MK3, {FrameId::THEIR_ROBOT_2}}};
    config.default_frame_id = FrameId::THEIR_ROBOT_2;
    config.max_jump_distance = 1.0;
    config.max_consecutive_jump_rejects = 1;
    config.robot_keypoint_tracker_config.min_length_meters = 0.05;
    config.robot_keypoint_tracker_config.max_length_meters = 2.0;
    config.robot_keypoint_tracker_config.min_confidence = 0.2;
    config.robot_keypoint_tracker_config.max_candidates = 8;

    RobotFrontBackSimpleFilter filter(config);
    ASSERT_TRUE(filter.initialize(2));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const KeypointsStamped empty_keypoints;
    const CommandFeedback command_feedback;

    KeypointsStamped blob_keypoints;
    blob_keypoints.header.frame_id = FrameId::CAMERA;
    blob_keypoints.header.stamp = 1.0;
    // Two generic OPPONENT blobs (could each take any of {THEIR_ROBOT_1, THEIR_ROBOT_2}).
    append_keypoint_pair(blob_keypoints, Label::OPPONENT, KeypointLabel::OPPONENT_FRONT,
                         KeypointLabel::OPPONENT_BACK, 1, 200.0, 240.0, 240.0, 0.7);
    append_keypoint_pair(blob_keypoints, Label::OPPONENT, KeypointLabel::OPPONENT_FRONT,
                         KeypointLabel::OPPONENT_BACK, 2, 400.0, 440.0, 240.0, 0.6);
    // One specific MRS_BUFF_MK3 blob (only allowed in THEIR_ROBOT_2). Highest confidence wins
    // tiebreakers under the global greedy.
    append_keypoint_pair(blob_keypoints, Label::MRS_BUFF_MK3, KeypointLabel::MRS_BUFF_MK3_FRONT,
                         KeypointLabel::MRS_BUFF_MK3_BACK, 3, 300.0, 340.0, 240.0, 0.95);

    const auto result =
        filter.update(empty_keypoints, field, camera_info, blob_keypoints, command_feedback);

    // Two slots, three blobs -- one blob is dropped. The MRS_BUFF_MK3 blob must keep its
    // configured slot and label; the higher-confidence OPPONENT blob takes the remaining slot.
    ASSERT_EQ(result.descriptions.size(), 2u);

    const RobotDescription *mk3_meas = nullptr;
    const RobotDescription *opponent_meas = nullptr;
    for (const auto &desc : result.descriptions) {
        if (desc.frame_id == FrameId::THEIR_ROBOT_2) mk3_meas = &desc;
        if (desc.frame_id == FrameId::THEIR_ROBOT_1) opponent_meas = &desc;
    }
    ASSERT_NE(mk3_meas, nullptr);
    ASSERT_NE(opponent_meas, nullptr);

    // MRS_BUFF_MK3 was assigned to its own configured FrameId, no fallback used, label kept.
    EXPECT_EQ(mk3_meas->label, Label::MRS_BUFF_MK3);
    EXPECT_EQ(mk3_meas->group, Group::THEIRS);

    // Remaining OPPONENT blob lands in THEIR_ROBOT_1, label unchanged.
    EXPECT_EQ(opponent_meas->label, Label::OPPONENT);
    EXPECT_EQ(opponent_meas->group, Group::THEIRS);
}
}  // namespace auto_battlebot
