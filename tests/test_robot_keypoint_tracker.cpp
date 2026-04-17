#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "robot_filter/robot_keypoint_tracker.hpp"

namespace auto_battlebot {
class RobotKeypointTrackerTest : public ::testing::Test {
   protected:
    RobotKeypointTrackerConfig config_;
    CameraInfo camera_info_;
    FieldDescription field_;

    void SetUp() override {
        config_.min_length_meters = 0.05;
        config_.max_length_meters = 2.0;
        config_.min_confidence = 0.2;
        config_.persistence_frames_required = 1;
        config_.match_distance_meters = 0.5;
        config_.tracked_timeout_seconds = 1.0;
        config_.max_candidates = 8;

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

    KeypointsStamped make_blob_keypoints() const {
        KeypointsStamped keypoints;
        keypoints.header.frame_id = FrameId::CAMERA;
        keypoints.header.stamp = 1.0;

        Keypoint a;
        a.label = Label::OPPONENT;
        a.keypoint_label = KeypointLabel::OPPONENT_FRONT;
        a.x = 300.0;
        a.y = 220.0;
        a.confidence = 0.9;
        a.detection_index = 1;
        keypoints.keypoints.push_back(a);

        Keypoint b = a;
        b.keypoint_label = KeypointLabel::OPPONENT_BACK;
        b.x = 340.0;
        b.y = 220.0;
        keypoints.keypoints.push_back(b);
        return keypoints;
    }
};

TEST_F(RobotKeypointTrackerTest, EmptyInputReturnsNothing) {
    RobotKeypointTracker tracker(config_);
    KeypointsStamped empty;
    auto detections = tracker.detect(empty, field_, camera_info_, 1.0);
    EXPECT_TRUE(detections.empty());
}

TEST_F(RobotKeypointTrackerTest, BlobPairProducesDetection) {
    RobotKeypointTracker tracker(config_);
    auto detections = tracker.detect(make_blob_keypoints(), field_, camera_info_, 1.0);
    ASSERT_EQ(detections.size(), 1u);
    EXPECT_EQ(detections[0].label, Label::OPPONENT);
    EXPECT_EQ(detections[0].group, Group::THEIRS);
    EXPECT_GT(detections[0].size.x, 0.05);
}

TEST_F(RobotKeypointTrackerTest, PersistenceFiltersEarlyFrames) {
    config_.persistence_frames_required = 2;
    RobotKeypointTracker tracker(config_);
    EXPECT_TRUE(tracker.detect(make_blob_keypoints(), field_, camera_info_, 1.0).empty());
    EXPECT_EQ(tracker.detect(make_blob_keypoints(), field_, camera_info_, 1.05).size(), 1u);
}
}  // namespace auto_battlebot
