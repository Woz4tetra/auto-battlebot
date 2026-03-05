#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "data_structures/camera.hpp"
#include "data_structures/field.hpp"
#include "data_structures/robot.hpp"
#include "robot_filter/floor_mask_detector.hpp"

namespace auto_battlebot {

class FloorMaskDetectorTest : public ::testing::Test {
   protected:
    FloorMaskDetectorConfig config_;
    CameraInfo camera_info_;
    FieldDescription field_;

    void SetUp() override {
        config_.min_blob_area_pixels = 10;
        config_.persistence_frames_required = 1;
        config_.blob_match_distance_meters = 0.5;
        config_.tracked_blob_timeout_seconds = 1.0;

        camera_info_.width = 640;
        camera_info_.height = 480;
        camera_info_.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        camera_info_.intrinsics.at<double>(0, 0) = 500.0;  // fx
        camera_info_.intrinsics.at<double>(1, 1) = 500.0;  // fy
        camera_info_.intrinsics.at<double>(0, 2) = 320.0;  // cx
        camera_info_.intrinsics.at<double>(1, 2) = 240.0;  // cy

        field_.child_frame_id = FrameId::FIELD;
        field_.size.size.x = 2.0;
        field_.size.size.y = 2.0;

        // Camera looking straight down at the field center from 3m height.
        // tf_camera_from_fieldcenter maps field coords → camera coords.
        // Camera convention: Z = forward (optical axis), X = right, Y = down.
        // For overhead view: field X → cam X, field Y → cam Y, field Z → cam -Z.
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf(0, 0) = 1;  tf(0, 1) = 0;  tf(0, 2) = 0;  tf(0, 3) = 0;
        tf(1, 0) = 0;  tf(1, 1) = 1;  tf(1, 2) = 0;  tf(1, 3) = 0;
        tf(2, 0) = 0;  tf(2, 1) = 0;  tf(2, 2) = -1; tf(2, 3) = 3.0;
        tf(3, 0) = 0;  tf(3, 1) = 0;  tf(3, 2) = 0;  tf(3, 3) = 1;
        field_.tf_camera_from_fieldcenter.tf = tf;
    }
};

TEST_F(FloorMaskDetectorTest, EmptyMaskReturnsNothing) {
    FloorMaskDetector detector(config_);
    cv::Mat empty_mask;
    auto results = detector.detect(empty_mask, field_, camera_info_, {}, 0.0);
    EXPECT_TRUE(results.empty());
}

TEST_F(FloorMaskDetectorTest, EmptyFieldReturnsNothing) {
    FloorMaskDetector detector(config_);
    cv::Mat mask = cv::Mat::ones(480, 640, CV_8UC1);
    FieldDescription empty_field;
    auto results = detector.detect(mask, empty_field, camera_info_, {}, 0.0);
    EXPECT_TRUE(results.empty());
}

TEST_F(FloorMaskDetectorTest, AllFloorReturnsNothing) {
    FloorMaskDetector detector(config_);
    // Mask: everything is floor (class 1)
    cv::Mat mask = cv::Mat::ones(480, 640, CV_8UC1);
    auto results = detector.detect(mask, field_, camera_info_, {}, 0.0);
    EXPECT_TRUE(results.empty());
}

TEST_F(FloorMaskDetectorTest, NotFloorBlobDetected) {
    FloorMaskDetector detector(config_);
    // Mask: everything is floor except a 30x30 block near center
    cv::Mat mask = cv::Mat::ones(480, 640, CV_8UC1);
    cv::Rect blob_rect(300, 220, 30, 30);
    mask(blob_rect) = 0;

    auto results = detector.detect(mask, field_, camera_info_, {}, 0.0);
    EXPECT_GE(results.size(), 1u);
    if (!results.empty()) {
        EXPECT_EQ(results[0].label, Label::OPPONENT);
    }
}

TEST_F(FloorMaskDetectorTest, PersistenceFilterRequiresMultipleFrames) {
    FloorMaskDetectorConfig strict_config = config_;
    strict_config.persistence_frames_required = 3;
    FloorMaskDetector detector(strict_config);

    cv::Mat mask = cv::Mat::ones(480, 640, CV_8UC1);
    cv::Rect blob_rect(300, 220, 30, 30);
    mask(blob_rect) = 0;

    // First two frames: not enough persistence
    auto r1 = detector.detect(mask, field_, camera_info_, {}, 0.0);
    EXPECT_TRUE(r1.empty());
    auto r2 = detector.detect(mask, field_, camera_info_, {}, 0.033);
    EXPECT_TRUE(r2.empty());

    // Third frame: blob should be persistent enough
    auto r3 = detector.detect(mask, field_, camera_info_, {}, 0.066);
    EXPECT_GE(r3.size(), 1u);
}

TEST_F(FloorMaskDetectorTest, ResetClearsState) {
    FloorMaskDetector detector(config_);

    cv::Mat mask = cv::Mat::ones(480, 640, CV_8UC1);
    cv::Rect blob_rect(300, 220, 30, 30);
    mask(blob_rect) = 0;

    detector.detect(mask, field_, camera_info_, {}, 0.0);
    detector.reset();

    FloorMaskDetectorConfig strict_config = config_;
    strict_config.persistence_frames_required = 2;
    FloorMaskDetector detector2(strict_config);
    auto r1 = detector2.detect(mask, field_, camera_info_, {}, 0.0);
    // First frame after reset with persistence=2 → empty
    EXPECT_TRUE(r1.empty());
}

TEST_F(FloorMaskDetectorTest, SmallBlobsFiltered) {
    FloorMaskDetectorConfig strict_config = config_;
    strict_config.min_blob_area_pixels = 1000;
    FloorMaskDetector detector(strict_config);

    cv::Mat mask = cv::Mat::ones(480, 640, CV_8UC1);
    // A 5x5 blob (area=25) is below the 1000 pixel threshold
    cv::Rect blob_rect(300, 220, 5, 5);
    mask(blob_rect) = 0;

    auto results = detector.detect(mask, field_, camera_info_, {}, 0.0);
    EXPECT_TRUE(results.empty());
}

}  // namespace auto_battlebot
