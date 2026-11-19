#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core.hpp>

#include "plant/mrs_buff_mk3_params.hpp"
#include "robot_filter/robot_front_back_filter.hpp"

namespace auto_battlebot {
namespace {
RobotDescriptionsStamped run_filter(RobotFrontBackFilter &filter, ModelResultStamped keypoints,
                                    const FieldDescription &field, const CameraInfo &camera_info,
                                    ModelResultStamped robot_blob_keypoints,
                                    const CommandFeedback &command_feedback) {
    filter.predict(keypoints.header.stamp, command_feedback);
    filter.correct(std::move(keypoints), field, camera_info, std::move(robot_blob_keypoints));
    return filter.state();
}

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

ModelResultStamped make_opponent_keypoints(double stamp, double front_x, double back_x) {
    ModelResultStamped keypoints;
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

void append_keypoint_pair(ModelResultStamped &model_results, Label label,
                          KeypointLabel front_kp_label, KeypointLabel back_kp_label,
                          int detection_index, double front_x, double back_x, double y,
                          double confidence) {
    Keypoint front;
    front.label = label;
    front.keypoint_label = front_kp_label;
    front.x = front_x;
    front.y = y;
    front.confidence = confidence;
    front.detection_index = detection_index;
    model_results.keypoints.push_back(front);

    Keypoint back = front;
    back.keypoint_label = back_kp_label;
    back.x = back_x;
    back.confidence = confidence;
    model_results.keypoints.push_back(back);
}

// Empty keypoint frame carrying a timestamp, so a keypoint-dropout frame still advances the
// filter clock (the output stamp is taken from keypoints.header.stamp).
ModelResultStamped make_empty_keypoints(double stamp) {
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;
    keypoints.header.stamp = stamp;
    return keypoints;
}

ModelResultStamped make_our_keypoints(double stamp, double front_x, double back_x, double y) {
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;
    keypoints.header.stamp = stamp;
    append_keypoint_pair(keypoints, Label::MRS_BUFF_MK3, KeypointLabel::MRS_BUFF_MK3_FRONT,
                         KeypointLabel::MRS_BUFF_MK3_BACK, 1, front_x, back_x, y, 0.9);
    return keypoints;
}

ModelResultStamped make_opponent_blob(double stamp, double front_x, double back_x, double y) {
    ModelResultStamped blob;
    blob.header.frame_id = FrameId::CAMERA;
    blob.header.stamp = stamp;
    append_keypoint_pair(blob, Label::OPPONENT, KeypointLabel::OPPONENT_FRONT,
                         KeypointLabel::OPPONENT_BACK, 1, front_x, back_x, y, 0.8);
    return blob;
}

RobotFrontBackFilterConfiguration make_our_robot_config() {
    RobotFrontBackFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT, KeypointLabel::MRS_BUFF_MK3_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK, KeypointLabel::MRS_BUFF_MK3_BACK};
    config.label_to_frame_ids = {{Label::MRS_BUFF_MK3, {FrameId::OUR_ROBOT_1}}};
    config.default_frame_id = FrameId::OUR_ROBOT_1;
    config.max_jump_distance = 1.0;
    config.max_consecutive_jump_rejects = 1;
    config.blob_overwrite_min_distance_meters = 0.3;
    config.blob_overwrite_size_scale = 0.5;
    config.robot_keypoint_tracker_config.min_length_meters = 0.05;
    config.robot_keypoint_tracker_config.max_length_meters = 2.0;
    config.robot_keypoint_tracker_config.min_confidence = 0.2;
    config.robot_keypoint_tracker_config.max_candidates = 8;
    return config;
}
}  // namespace

TEST(RobotFrontBackFilterTest, RejectsLargeJumpThenAcceptsAfterThreshold) {
    RobotFrontBackFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK};
    config.label_to_frame_ids = {{Label::OPPONENT, {FrameId::THEIR_ROBOT_1}}};
    config.default_frame_id = FrameId::THEIR_ROBOT_1;
    config.max_jump_distance = 0.2;
    config.max_consecutive_jump_rejects = 1;

    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped empty_blob_keypoints;
    const CommandFeedback command_feedback;

    const auto first = run_filter(filter, make_opponent_keypoints(1.0, 300.0, 340.0), field,
                                  camera_info, empty_blob_keypoints, command_feedback);
    ASSERT_EQ(first.descriptions.size(), 1u);
    const double first_x = first.descriptions[0].pose.position.x;
    EXPECT_FALSE(first.descriptions[0].is_stale);

    const auto second = run_filter(filter, make_opponent_keypoints(1.1, 460.0, 500.0), field,
                                   camera_info, empty_blob_keypoints, command_feedback);
    ASSERT_EQ(second.descriptions.size(), 1u);
    EXPECT_TRUE(second.descriptions[0].is_stale);
    EXPECT_NEAR(second.descriptions[0].pose.position.x, first_x, 1e-6);

    const auto third = run_filter(filter, make_opponent_keypoints(1.2, 460.0, 500.0), field,
                                  camera_info, empty_blob_keypoints, command_feedback);
    ASSERT_EQ(third.descriptions.size(), 1u);
    EXPECT_FALSE(third.descriptions[0].is_stale);
    EXPECT_GT(std::abs(third.descriptions[0].pose.position.x - first_x), 0.5);
}

// when a specific THEIRS-label blob can't be assigned to its own configured
// FrameId (because a keypoint detection took it) and falls back to a generic OPPONENT slot,
// the resulting RobotDescription must be re-labeled to Label::OPPONENT so (label, frame_id,
// group) stay consistent. Otherwise downstream consumers see a specific-model label sitting
// in a generic opponent slot, and the label flips back to its specific value the next tick a
// "proper" slot frees up (an ID swap).
TEST(RobotFrontBackFilterTest, BlobFallbackToOpponentRelabelsToOpponent) {
    RobotFrontBackFilterConfiguration config;
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

    RobotFrontBackFilter filter(config);
    // 2 opponents -> Label::OPPONENT maps to {THEIR_ROBOT_1, THEIR_ROBOT_2}, so MRS_BUFF_MK1's
    // own slot {THEIR_ROBOT_1} can be exhausted by an OPPONENT keypoint while a free OPPONENT
    // fallback slot remains.
    ASSERT_TRUE(filter.initialize(2));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const CommandFeedback command_feedback;

    const ModelResultStamped opponent_keypoints = make_opponent_keypoints(1.0, 280.0, 320.0);

    // MRS_BUFF_MK1 blob, spatially separated from the keypoint detection so it survives the
    // is_blob_suppressed_by_keypoint filter. It can only land in THEIR_ROBOT_2 via the
    // THEIRS-fallback to OPPONENT slots.
    ModelResultStamped blob_keypoints;
    blob_keypoints.header.frame_id = FrameId::CAMERA;
    blob_keypoints.header.stamp = 1.0;
    append_keypoint_pair(blob_keypoints, Label::MRS_BUFF_MK1, KeypointLabel::MRS_BUFF_MK1_FRONT,
                         KeypointLabel::MRS_BUFF_MK1_BACK, 1, 430.0, 470.0, 240.0, 0.8);

    const auto result = run_filter(filter, opponent_keypoints, field, camera_info, blob_keypoints,
                                   command_feedback);
    ASSERT_EQ(result.descriptions.size(), 2u);

    const RobotDescription *kp_meas = nullptr;
    const RobotDescription *blob_meas = nullptr;
    for (const auto &desc : result.descriptions) {
        if (desc.frame_id == FrameId::THEIR_ROBOT_1) kp_meas = &desc;
        if (desc.frame_id == FrameId::THEIR_ROBOT_2) blob_meas = &desc;
    }
    ASSERT_NE(kp_meas, nullptr);
    ASSERT_NE(blob_meas, nullptr);

    EXPECT_EQ(kp_meas->label, Label::OPPONENT);
    EXPECT_EQ(kp_meas->group, Group::THEIRS);

    EXPECT_EQ(blob_meas->label, Label::OPPONENT);
    EXPECT_EQ(blob_meas->group, Group::THEIRS);
}

TEST(RobotFrontBackFilterTest, GlobalAssignmentDoesNotDropSpecificLabelBlob) {
    RobotFrontBackFilterConfiguration config;
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

    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(2));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped empty_keypoints;
    const CommandFeedback command_feedback;

    ModelResultStamped blob_keypoints;
    blob_keypoints.header.frame_id = FrameId::CAMERA;
    blob_keypoints.header.stamp = 1.0;
    append_keypoint_pair(blob_keypoints, Label::OPPONENT, KeypointLabel::OPPONENT_FRONT,
                         KeypointLabel::OPPONENT_BACK, 1, 200.0, 240.0, 240.0, 0.7);
    append_keypoint_pair(blob_keypoints, Label::OPPONENT, KeypointLabel::OPPONENT_FRONT,
                         KeypointLabel::OPPONENT_BACK, 2, 400.0, 440.0, 240.0, 0.6);
    // One specific MRS_BUFF_MK3 blob (only allowed in THEIR_ROBOT_2). Highest confidence wins
    // tiebreakers under the global greedy.
    append_keypoint_pair(blob_keypoints, Label::MRS_BUFF_MK3, KeypointLabel::MRS_BUFF_MK3_FRONT,
                         KeypointLabel::MRS_BUFF_MK3_BACK, 3, 300.0, 340.0, 240.0, 0.95);

    const auto result =
        run_filter(filter, empty_keypoints, field, camera_info, blob_keypoints, command_feedback);

    ASSERT_EQ(result.descriptions.size(), 2u);

    const RobotDescription *mk3_meas = nullptr;
    const RobotDescription *opponent_meas = nullptr;
    for (const auto &desc : result.descriptions) {
        if (desc.frame_id == FrameId::THEIR_ROBOT_2) mk3_meas = &desc;
        if (desc.frame_id == FrameId::THEIR_ROBOT_1) opponent_meas = &desc;
    }
    ASSERT_NE(mk3_meas, nullptr);
    ASSERT_NE(opponent_meas, nullptr);

    EXPECT_EQ(mk3_meas->label, Label::MRS_BUFF_MK3);
    EXPECT_EQ(mk3_meas->group, Group::THEIRS);

    EXPECT_EQ(opponent_meas->label, Label::OPPONENT);
    EXPECT_EQ(opponent_meas->group, Group::THEIRS);
}

TEST(RobotFrontBackFilterTest, SharedSlotGoesToHigherConfidenceBlobWhicheverLabelItHas) {
    const auto winner_x = [](double mk1_confidence, double mk3_confidence) {
        RobotFrontBackFilterConfiguration config;
        config.front_keypoints = {KeypointLabel::MRS_BUFF_MK1_FRONT,
                                  KeypointLabel::MRS_BUFF_MK3_FRONT};
        config.back_keypoints = {KeypointLabel::MRS_BUFF_MK1_BACK,
                                 KeypointLabel::MRS_BUFF_MK3_BACK};
        config.label_to_frame_ids = {{Label::MRS_BUFF_MK1, {FrameId::THEIR_ROBOT_1}},
                                     {Label::MRS_BUFF_MK3, {FrameId::THEIR_ROBOT_1}}};
        config.default_frame_id = FrameId::THEIR_ROBOT_1;
        config.max_jump_distance = 1.0;
        config.max_consecutive_jump_rejects = 1;
        config.robot_keypoint_tracker_config.min_length_meters = 0.05;
        config.robot_keypoint_tracker_config.max_length_meters = 2.0;
        config.robot_keypoint_tracker_config.min_confidence = 0.2;
        config.robot_keypoint_tracker_config.max_candidates = 8;
        RobotFrontBackFilter filter(config);
        EXPECT_TRUE(filter.initialize(1));

        ModelResultStamped blob_keypoints;
        blob_keypoints.header.frame_id = FrameId::CAMERA;
        blob_keypoints.header.stamp = 1.0;
        append_keypoint_pair(blob_keypoints, Label::MRS_BUFF_MK1, KeypointLabel::MRS_BUFF_MK1_FRONT,
                             KeypointLabel::MRS_BUFF_MK1_BACK, 1, 200.0, 240.0, 240.0,
                             mk1_confidence);
        append_keypoint_pair(blob_keypoints, Label::MRS_BUFF_MK3, KeypointLabel::MRS_BUFF_MK3_FRONT,
                             KeypointLabel::MRS_BUFF_MK3_BACK, 2, 400.0, 440.0, 240.0,
                             mk3_confidence);

        const auto result = run_filter(filter, ModelResultStamped{}, make_field(),
                                       make_camera_info(), blob_keypoints, CommandFeedback{});
        EXPECT_EQ(result.descriptions.size(), 1u);
        return result.descriptions.empty() ? 0.0 : result.descriptions.front().pose.position.x;
    };

    EXPECT_NE(winner_x(0.9, 0.5), winner_x(0.5, 0.9))
        << "the winner follows confidence, not which label is first in the enum";
}

TEST(RobotFrontBackFilterTest, FlagsLeakOpportunityOnKeypointMiss) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    const auto first = run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field,
                                  camera_info, no_blob, command_feedback);
    ASSERT_EQ(first.descriptions.size(), 1u);
    ASSERT_EQ(first.descriptions[0].frame_id, FrameId::OUR_ROBOT_1);
    EXPECT_FALSE(filter.last_suppressed_own_blob());

    // Frame 2: our keypoint drops out; a blob sits where our robot was. This is a leak-opportunity.
    // The frame stamp must come from the keypoints, which drive the filter's clock.
    const auto second = run_filter(filter, make_empty_keypoints(1.1), field, camera_info,
                                   make_opponent_blob(1.1, 300.0, 340.0, 220.0), command_feedback);
    EXPECT_TRUE(filter.last_suppressed_own_blob());
    (void)second;
}

TEST(RobotFrontBackFilterTest, NoLeakFlagWhenBlobFarFromHeldPose) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);

    // Keypoint miss, but the blob is well away from the held pose (far in image y).
    const ModelResultStamped no_keypoints;
    run_filter(filter, no_keypoints, field, camera_info,
               make_opponent_blob(1.1, 300.0, 340.0, 380.0), command_feedback);
    EXPECT_FALSE(filter.last_suppressed_own_blob());
}

TEST(RobotFrontBackFilterTest, NoLeakFlagWhenOurKeypointPresent) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);

    // Our keypoint is present again this frame alongside a coincident blob: not a leak-opportunity.
    run_filter(filter, make_our_keypoints(1.1, 300.0, 340.0, 220.0), field, camera_info,
               make_opponent_blob(1.1, 300.0, 340.0, 220.0), command_feedback);
    EXPECT_FALSE(filter.last_suppressed_own_blob());
}

TEST(RobotFrontBackFilterTest, DecaysStaleOurRobotAfterHoldWindow) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    config.our_robot_hold_window_s = 0.15;
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    const auto first = run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field,
                                  camera_info, no_blob, command_feedback);
    ASSERT_EQ(first.descriptions.size(), 1u);
    ASSERT_EQ(first.descriptions[0].frame_id, FrameId::OUR_ROBOT_1);
    EXPECT_FALSE(first.descriptions[0].is_stale);

    // Frame 2: keypoint miss 0.10 s later (within the 0.15 s window) -> still held, flagged stale.
    const auto within = run_filter(filter, make_empty_keypoints(1.10), field, camera_info, no_blob,
                                   command_feedback);
    ASSERT_EQ(within.descriptions.size(), 1u);
    EXPECT_EQ(within.descriptions[0].frame_id, FrameId::OUR_ROBOT_1);
    EXPECT_TRUE(within.descriptions[0].is_stale);

    // Frame 3: still no keypoint, now 0.20 s past the last confirmation (> window) -> dropped.
    const auto expired = run_filter(filter, make_empty_keypoints(1.20), field, camera_info, no_blob,
                                    command_feedback);
    EXPECT_TRUE(expired.descriptions.empty());
}

TEST(RobotFrontBackFilterTest, HoldsStaleOurRobotWhenDecayDisabled) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    config.our_robot_hold_window_s = 0.0;
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);

    // A full second later with no keypoints: still held (stale), never dropped.
    const auto held = run_filter(filter, make_empty_keypoints(2.0), field, camera_info, no_blob,
                                 command_feedback);
    ASSERT_EQ(held.descriptions.size(), 1u);
    EXPECT_EQ(held.descriptions[0].frame_id, FrameId::OUR_ROBOT_1);
    EXPECT_TRUE(held.descriptions[0].is_stale);
}

namespace {
const RobotDescription *find_frame_id(const RobotDescriptionsStamped &result, FrameId frame_id) {
    for (const auto &description : result.descriptions) {
        if (description.frame_id == frame_id) return &description;
    }
    return nullptr;
}
}  // namespace

// The blob model has no class for our robot, so during a keypoint dropout a blob sitting on our
// own position would be assigned an opponent FrameId and targeted. It must be discarded instead.
TEST(RobotFrontBackFilterTest, SuppressesBlobOnOurHeldPoseDuringKeypointDropout) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);

    // Frame 2: our keypoints drop out; a blob lands on the same spot.
    const auto result = run_filter(filter, make_empty_keypoints(1.1), field, camera_info,
                                   make_opponent_blob(1.1, 300.0, 340.0, 220.0), command_feedback);

    EXPECT_TRUE(filter.last_suppressed_own_blob());
    EXPECT_EQ(find_frame_id(result, FrameId::THEIR_ROBOT_1), nullptr);
}

TEST(RobotFrontBackFilterTest, AnchorOutlivesDroppedOurTrack) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    config.our_robot_hold_window_s = 0.15;
    config.our_keypoint_dropout_blob_window_s = 1.0;
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);
    const auto dropped = run_filter(filter, make_empty_keypoints(1.3), field, camera_info,
                                    make_opponent_blob(1.3, 300.0, 340.0, 220.0), command_feedback);

    EXPECT_EQ(find_frame_id(dropped, FrameId::OUR_ROBOT_1), nullptr);

    const auto later = run_filter(filter, make_empty_keypoints(1.5), field, camera_info,
                                  make_opponent_blob(1.5, 300.0, 340.0, 220.0), command_feedback);
    EXPECT_TRUE(filter.last_suppressed_own_blob());
    EXPECT_EQ(find_frame_id(later, FrameId::THEIR_ROBOT_1), nullptr);
}

// The suppression is time-bounded: once the window elapses the anchor is no longer trusted, so a
// genuine opponent parked where we were last seen becomes targetable again.
TEST(RobotFrontBackFilterTest, EmitsBlobOnOurHeldPoseAfterWindowExpires) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    config.our_keypoint_dropout_blob_window_s = 0.4;
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);

    // Well past the window, and past our_robot_hold_window_s so the anchor is frozen, not tracked.
    const auto result = run_filter(filter, make_empty_keypoints(2.0), field, camera_info,
                                   make_opponent_blob(2.0, 300.0, 340.0, 220.0), command_feedback);

    EXPECT_FALSE(filter.last_suppressed_own_blob());
    EXPECT_NE(find_frame_id(result, FrameId::THEIR_ROBOT_1), nullptr);
}

TEST(RobotFrontBackFilterTest, KeepsBlobOutsideRadiusDuringKeypointDropout) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);

    // Same dropout, but the blob is ~0.96 m away in the field frame, well outside the radius.
    const auto result = run_filter(filter, make_empty_keypoints(1.1), field, camera_info,
                                   make_opponent_blob(1.1, 300.0, 340.0, 380.0), command_feedback);

    EXPECT_FALSE(filter.last_suppressed_own_blob());
    EXPECT_NE(find_frame_id(result, FrameId::THEIR_ROBOT_1), nullptr);
}

TEST(RobotFrontBackFilterTest, ZeroRadiusDisablesDropoutSuppression) {
    RobotFrontBackFilterConfiguration config = make_our_robot_config();
    config.our_keypoint_dropout_blob_radius_meters = 0.0;
    RobotFrontBackFilter filter(config);
    ASSERT_TRUE(filter.initialize(1));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const ModelResultStamped no_blob;
    const CommandFeedback command_feedback;

    run_filter(filter, make_our_keypoints(1.0, 300.0, 340.0, 220.0), field, camera_info, no_blob,
               command_feedback);

    const auto result = run_filter(filter, make_empty_keypoints(1.1), field, camera_info,
                                   make_opponent_blob(1.1, 300.0, 340.0, 220.0), command_feedback);

    EXPECT_FALSE(filter.last_suppressed_own_blob());
    EXPECT_NE(find_frame_id(result, FrameId::THEIR_ROBOT_1), nullptr);
}
RobotFrontBackFilterConfiguration make_kalman_opponent_config() {
    RobotFrontBackFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK};
    config.label_to_frame_ids = {{Label::OPPONENT, {FrameId::THEIR_ROBOT_1}}};
    config.default_frame_id = FrameId::THEIR_ROBOT_1;
    config.max_jump_distance = 1.0;
    config.robot_keypoint_tracker_config.min_length_meters = 0.05;
    config.robot_keypoint_tracker_config.max_length_meters = 2.0;
    config.robot_keypoint_tracker_config.min_confidence = 0.2;
    config.robot_keypoint_tracker_config.max_candidates = 8;
    auto kalman = std::make_shared<KalmanMotionEstimatorConfiguration>();
    kalman->opponent_mode = OpponentMode::KALMAN;
    kalman->opponent_max_coast_s = 1.0;
    kalman->plant = mrs_buff_mk3_plant();
    config.motion_estimator = kalman;
    return config;
}

constexpr double kOpponentPixelsPerSecond = 80.0;
constexpr double kFrameDt = 1.0 / 30.0;
constexpr double kLateMeasurementShiftPixels = 12.0;

ModelResultStamped make_moving_opponent_keypoints(double stamp, double extra_pixels = 0.0) {
    const double offset = kOpponentPixelsPerSecond * (stamp - 1.0) + extra_pixels;
    return make_opponent_keypoints(stamp, 280.0 + offset, 320.0 + offset);
}

double drive_opponent_to(RobotFrontBackFilter &filter, const CameraInfo &camera_info,
                         const FieldDescription &field, int frames) {
    double stamp = 1.0;
    for (int i = 0; i < frames; ++i) {
        stamp = 1.0 + i * kFrameDt;
        run_filter(filter, make_moving_opponent_keypoints(stamp), field, camera_info,
                   ModelResultStamped{}, CommandFeedback{});
    }
    return stamp;
}

const RobotDescription *find_opponent(const RobotDescriptionsStamped &state) {
    for (const auto &description : state.descriptions) {
        if (description.frame_id == FrameId::THEIR_ROBOT_1) return &description;
    }
    return nullptr;
}

TEST(RobotFrontBackFilterTest, CorrectWithStampOlderThanPredict) {
    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    constexpr double kReadAfterCorrectS = 0.3;

    RobotFrontBackFilterConfiguration lockstep_config = make_kalman_opponent_config();
    RobotFrontBackFilter lockstep(lockstep_config);
    ASSERT_TRUE(lockstep.initialize(1));
    const double last = drive_opponent_to(lockstep, camera_info, field, 30);
    const double late_stamp = last + kFrameDt;
    lockstep.predict(late_stamp, CommandFeedback{});
    lockstep.correct(make_moving_opponent_keypoints(late_stamp, kLateMeasurementShiftPixels), field,
                     camera_info, ModelResultStamped{});
    lockstep.predict(late_stamp + kReadAfterCorrectS, CommandFeedback{});
    const RobotDescriptionsStamped expected = lockstep.state();

    RobotFrontBackFilterConfiguration outrun_config = make_kalman_opponent_config();
    RobotFrontBackFilter outrun(outrun_config);
    ASSERT_TRUE(outrun.initialize(1));
    drive_opponent_to(outrun, camera_info, field, 30);
    outrun.predict(late_stamp + 0.1, CommandFeedback{});
    outrun.predict(late_stamp + 0.2, CommandFeedback{});
    outrun.correct(make_moving_opponent_keypoints(late_stamp, kLateMeasurementShiftPixels), field,
                   camera_info, ModelResultStamped{});
    outrun.predict(late_stamp + kReadAfterCorrectS, CommandFeedback{});
    const RobotDescriptionsStamped actual = outrun.state();

    const RobotDescription *expected_opponent = find_opponent(expected);
    const RobotDescription *actual_opponent = find_opponent(actual);
    ASSERT_NE(expected_opponent, nullptr);
    ASSERT_NE(actual_opponent, nullptr);
    EXPECT_NEAR(actual_opponent->pose.position.x, expected_opponent->pose.position.x, 0.005);
    EXPECT_NEAR(actual_opponent->pose.position.y, expected_opponent->pose.position.y, 0.005);
}

}  // namespace auto_battlebot
