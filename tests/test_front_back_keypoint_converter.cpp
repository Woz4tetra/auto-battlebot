#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "robot_filter/front_back_keypoint_converter.hpp"

namespace auto_battlebot {
class FrontBackKeypointConverterTest : public ::testing::Test {
   protected:
    FrontBackKeypointConverterConfig config_;
    CameraInfo camera_info_;
    FieldDescription field_;

    void SetUp() override {
        config_.front_keypoints = {KeypointLabel::OPPONENT_FRONT};
        config_.back_keypoints = {KeypointLabel::OPPONENT_BACK};

        camera_info_.width = 640;
        camera_info_.height = 480;
        camera_info_.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        camera_info_.intrinsics.at<double>(0, 0) = 500.0;
        camera_info_.intrinsics.at<double>(1, 1) = 500.0;
        camera_info_.intrinsics.at<double>(0, 2) = 320.0;
        camera_info_.intrinsics.at<double>(1, 2) = 240.0;

        field_.child_frame_id = FrameId::FIELD;
        field_.size.size.x = 2.0;
        field_.size.size.y = 2.0;
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf(2, 2) = -1.0;
        tf(2, 3) = 3.0;
        field_.tf_camera_from_fieldcenter.tf = tf;
    }

    Keypoint make_keypoint(KeypointLabel keypoint_label, int detection_index, double confidence,
                           double x = 320.0, double y = 220.0) const {
        Keypoint keypoint;
        keypoint.label = Label::OPPONENT;
        keypoint.keypoint_label = keypoint_label;
        keypoint.x = x;
        keypoint.y = y;
        keypoint.confidence = confidence;
        keypoint.detection_index = detection_index;
        return keypoint;
    }
};

TEST_F(FrontBackKeypointConverterTest, EmitsOnlyCompleteFrontBackAssignments) {
    FrontBackKeypointConverter converter(config_);
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;

    keypoints.keypoints.push_back(
        make_keypoint(KeypointLabel::OPPONENT_FRONT, 1, 0.9, 300.0, 220.0));
    keypoints.keypoints.push_back(
        make_keypoint(KeypointLabel::OPPONENT_BACK, 1, 0.8, 340.0, 220.0));
    keypoints.keypoints.push_back(
        make_keypoint(KeypointLabel::OPPONENT_FRONT, 2, 0.7, 360.0, 220.0));

    const auto mapping = converter.convert(keypoints, field_, camera_info_);
    auto it = mapping.find(Label::OPPONENT);
    ASSERT_NE(it, mapping.end());
    ASSERT_EQ(it->second.size(), 1u);
}

TEST_F(FrontBackKeypointConverterTest, UsesFirstKeypointConfidenceForAssignment) {
    FrontBackKeypointConverter converter(config_);
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;

    keypoints.keypoints.push_back(
        make_keypoint(KeypointLabel::OPPONENT_FRONT, 7, 0.2, 300.0, 220.0));
    keypoints.keypoints.push_back(
        make_keypoint(KeypointLabel::OPPONENT_BACK, 7, 0.9, 340.0, 220.0));

    const auto mapping = converter.convert(keypoints, field_, camera_info_);
    auto it = mapping.find(Label::OPPONENT);
    ASSERT_NE(it, mapping.end());
    ASSERT_EQ(it->second.size(), 1u);
    EXPECT_DOUBLE_EQ(it->second[0].second, 0.2);
}

TEST_F(FrontBackKeypointConverterTest, PoseSitsAtReferenceOffsetAlongHeading) {
    FrontBackKeypointConverter converter(config_);
    Transform tf;

    // Facing +X, keypoints at 0 and 1: the axle 0.3 m behind the midpoint lands at 0.2.
    ASSERT_TRUE(converter.get_pose_from_points(Eigen::Vector3d(1.0, 0.0, 0.0),
                                               Eigen::Vector3d::Zero(), -0.3, tf));
    EXPECT_NEAR(tf.tf(0, 3), 0.2, 1e-12);
    EXPECT_NEAR(tf.tf(1, 3), 0.0, 1e-12);

    // Facing +Y the shift follows the heading, not the field axes.
    ASSERT_TRUE(converter.get_pose_from_points(Eigen::Vector3d(0.0, 1.0, 0.0),
                                               Eigen::Vector3d::Zero(), 0.25, tf));
    EXPECT_NEAR(tf.tf(0, 3), 0.0, 1e-12);
    EXPECT_NEAR(tf.tf(1, 3), 0.75, 1e-12);
}

TEST_F(FrontBackKeypointConverterTest, AssignmentCarriesKeypointReferenceOffset) {
    FrontBackKeypointConverter converter(config_);
    ModelResultStamped keypoints;
    keypoints.header.frame_id = FrameId::CAMERA;
    for (Keypoint keypoint : {make_keypoint(KeypointLabel::OPPONENT_FRONT, 1, 0.9, 300.0, 220.0),
                              make_keypoint(KeypointLabel::OPPONENT_BACK, 1, 0.9, 340.0, 220.0)}) {
        keypoint.reference_offset_m = -0.0455;
        keypoints.keypoints.push_back(keypoint);
    }

    const auto mapping = converter.convert(keypoints, field_, camera_info_);
    const auto it = mapping.find(Label::OPPONENT);
    ASSERT_NE(it, mapping.end());
    ASSERT_EQ(it->second.size(), 1u);
    EXPECT_DOUBLE_EQ(it->second[0].first.reference_offset_m, -0.0455);
}
}  // namespace auto_battlebot
