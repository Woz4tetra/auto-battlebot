#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Dense>
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
}  // namespace

TEST(RobotFrontBackSimpleFilterTest, RejectsLargeJumpThenAcceptsAfterThreshold) {
    RobotFrontBackSimpleFilterConfiguration config;
    config.front_keypoints = {KeypointLabel::OPPONENT_FRONT};
    config.back_keypoints = {KeypointLabel::OPPONENT_BACK};
    config.label_to_frame_ids = {{Label::OPPONENT, {FrameId::THEIR_ROBOT_1}}};
    config.default_frame_id = FrameId::THEIR_ROBOT_1;
    config.max_jump_distance = 0.2;
    config.max_consecutive_jump_rejects = 1;

    RobotFrontBackSimpleFilter filter(config);
    ASSERT_TRUE(filter.initialize({RobotConfig{Label::OPPONENT, Group::THEIRS}}));

    const CameraInfo camera_info = make_camera_info();
    const FieldDescription field = make_field();
    const KeypointsStamped empty_blob_keypoints;
    const CommandFeedback command_feedback;

    const auto first = filter.update(make_opponent_keypoints(1.0, 300.0, 340.0), field, camera_info,
                                     empty_blob_keypoints, command_feedback);
    ASSERT_EQ(first.descriptions.size(), 1u);
    const double first_x = first.descriptions[0].pose.position.x;
    EXPECT_FALSE(first.descriptions[0].is_stale);

    const auto second = filter.update(make_opponent_keypoints(1.1, 460.0, 500.0), field, camera_info,
                                      empty_blob_keypoints, command_feedback);
    ASSERT_EQ(second.descriptions.size(), 1u);
    EXPECT_TRUE(second.descriptions[0].is_stale);
    EXPECT_NEAR(second.descriptions[0].pose.position.x, first_x, 1e-6);

    const auto third = filter.update(make_opponent_keypoints(1.2, 460.0, 500.0), field, camera_info,
                                     empty_blob_keypoints, command_feedback);
    ASSERT_EQ(third.descriptions.size(), 1u);
    EXPECT_FALSE(third.descriptions[0].is_stale);
    EXPECT_GT(std::abs(third.descriptions[0].pose.position.x - first_x), 0.5);
}
}  // namespace auto_battlebot
