#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <limits>
#include <opencv2/core.hpp>

#include "keypoint_filter/static_gate.hpp"

namespace auto_battlebot {

class StaticDetectionGateTest : public ::testing::Test {
   protected:
    static constexpr double kCameraHeight = 3.0;
    static constexpr double kFx = 500.0;
    static constexpr double kMetresPerPixelAt3m = kCameraHeight / kFx;
    inline static const cv::Point2d kFloorGraphicPx{400.0, 300.0};
    static constexpr double kFramePeriodS = 0.1;

    StaticDetectionGateConfiguration config_;
    CameraInfo camera_info_;
    FieldDescription field_;

    void SetUp() override {
        config_.enable = true;
        config_.match_radius_meters = 0.20;
        config_.static_radius_meters = 0.05;
        config_.responsiveness_window = 60;
        config_.min_dwell_seconds = 1.0;
        config_.min_observations = 5;
        config_.forget_seconds = 2.0;

        camera_info_.width = 640;
        camera_info_.height = 480;
        camera_info_.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        camera_info_.intrinsics.at<double>(0, 0) = kFx;
        camera_info_.intrinsics.at<double>(1, 1) = kFx;
        camera_info_.intrinsics.at<double>(0, 2) = 320.0;
        camera_info_.intrinsics.at<double>(1, 2) = 240.0;

        field_.child_frame_id = FrameId::FIELD;
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf(2, 2) = -1.0;
        tf(2, 3) = kCameraHeight;
        field_.tf_camera_from_fieldcenter.tf = tf;
    }

    static ModelResultStamped make_pair(double x, double y, double stamp) {
        ModelResultStamped keypoints;
        keypoints.header.frame_id = FrameId::CAMERA;
        keypoints.header.stamp = stamp;

        Keypoint a;
        a.label = Label::OPPONENT;
        a.keypoint_label = KeypointLabel::OPPONENT_FRONT;
        a.x = x - 10.0;
        a.y = y;
        a.confidence = 0.9;
        a.detection_index = 1;

        Keypoint b = a;
        b.keypoint_label = KeypointLabel::OPPONENT_BACK;
        b.x = x + 10.0;

        keypoints.keypoints = {a, b};
        keypoints.boxes.resize(2);
        keypoints.boxes[1] = BoundingBox{x - 30.0, y - 30.0, x + 30.0, y + 30.0};
        return keypoints;
    }
};

TEST_F(StaticDetectionGateTest, SuppressesADetectionThatNeverMoves) {
    StaticDetectionGate gate(config_);

    constexpr int kStaticFrames = 10;
    for (int i = 0; i < kStaticFrames; ++i) {
        const double stamp = kFramePeriodS * i;
        const auto result = gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp),
                                        camera_info_, field_, stamp);
        EXPECT_EQ(result.keypoints.size(), 2u) << "suppressed too early at stamp " << stamp;
    }

    const double stamp_past_dwell = config_.min_dwell_seconds;
    ASSERT_GE(kStaticFrames, config_.min_observations);
    const auto result =
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp_past_dwell), camera_info_,
                    field_, stamp_past_dwell);
    EXPECT_TRUE(result.keypoints.empty());
    EXPECT_EQ(gate.last_stats().suppressed, 1);
    EXPECT_EQ(gate.last_stats().static_clusters, 1);
}

TEST_F(StaticDetectionGateTest, NeverSuppressesAMovingDetection) {
    StaticDetectionGate gate(config_);

    constexpr double kDriftPxPerFrame = 4.0;
    constexpr int kFramesToClearStaticRadius = 3;
    ASSERT_GT(kDriftPxPerFrame * kMetresPerPixelAt3m * kFramesToClearStaticRadius,
              config_.static_radius_meters);
    for (int i = 0; i < 40; ++i) {
        const double stamp = 0.1 * i;
        const auto result = gate.filter(make_pair(200.0 + kDriftPxPerFrame * i, 300.0, stamp),
                                        camera_info_, field_, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u)
            << "suppressed a moving detection at stamp " << stamp;
    }
    EXPECT_EQ(gate.last_stats().suppressed, 0);
}

TEST_F(StaticDetectionGateTest, ARobotMovingInsideTheMatchRadiusIsNotScenery) {
    StaticDetectionGate gate(config_);

    constexpr double kSweepPx = 25.0;
    const double sweep_m = kSweepPx * kMetresPerPixelAt3m;
    ASSERT_LT(sweep_m, config_.match_radius_meters);
    ASSERT_GT(sweep_m / 2.0, config_.static_radius_meters);
    for (int i = 0; i < 60; ++i) {
        const double stamp = 0.1 * i;
        const double x = (i % 2 == 0) ? kFloorGraphicPx.x : kFloorGraphicPx.x + kSweepPx;
        const auto result = gate.filter(make_pair(x, 300.0, stamp), camera_info_, field_, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u) << "suppressed a moving robot at stamp " << stamp;
    }
    EXPECT_EQ(gate.last_stats().suppressed, 0);
}

TEST_F(StaticDetectionGateTest, ARobotArrivingOnALogoLiftsSuppressionThenItReturns) {
    StaticDetectionGate gate(config_);

    double stamp = 0.0;
    for (int i = 0; i < 60; ++i, stamp += 0.1) {
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp), camera_info_, field_,
                    stamp);
    }
    ASSERT_TRUE(gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp), camera_info_,
                            field_, stamp)
                    .keypoints.empty())
        << "the graphic should be suppressed by now";
    stamp += 0.1;

    constexpr double kRobotSweepLeftX = 385.0;
    constexpr double kRobotSweepRightX = 430.0;
    bool lifted = false;
    for (int i = 0; i < 40; ++i, stamp += 0.1) {
        const double x = (i % 2 == 0) ? kRobotSweepLeftX : kRobotSweepRightX;
        if (!gate.filter(make_pair(x, 300.0, stamp), camera_info_, field_, stamp)
                 .keypoints.empty()) {
            lifted = true;
        }
    }
    EXPECT_TRUE(lifted) << "a robot on a suppressed cluster stayed invisible";

    bool resuppressed = false;
    for (int i = 0; i < 120; ++i, stamp += 0.1) {
        if (gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp), camera_info_,
                        field_, stamp)
                .keypoints.empty()) {
            resuppressed = true;
        }
    }
    EXPECT_TRUE(resuppressed) << "suppression never recovered after the robot left";
}

TEST_F(StaticDetectionGateTest, ForgetsAClusterAfterAGap) {
    StaticDetectionGate gate(config_);

    for (int i = 0; i < 15; ++i) {
        const double stamp = 0.1 * i;
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp), camera_info_, field_,
                    stamp);
    }
    ASSERT_TRUE(
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, 1.5), camera_info_, field_, 1.5)
            .keypoints.empty());

    const double stamp_after_gap = 1.5 + config_.forget_seconds + 1.0;
    const auto result =
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp_after_gap), camera_info_,
                    field_, stamp_after_gap);
    EXPECT_EQ(result.keypoints.size(), 2u);
}

TEST_F(StaticDetectionGateTest, ResetClearsClustersOnFieldReinit) {
    StaticDetectionGate gate(config_);

    for (int i = 0; i < 15; ++i) {
        const double stamp = 0.1 * i;
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp), camera_info_, field_,
                    stamp);
    }
    ASSERT_TRUE(
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, 1.5), camera_info_, field_, 1.5)
            .keypoints.empty());

    gate.reset();
    const auto result = gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, 1.6),
                                    camera_info_, field_, 1.6);
    EXPECT_EQ(result.keypoints.size(), 2u);
}

TEST_F(StaticDetectionGateTest, DisabledGatePassesEverything) {
    config_.enable = false;
    StaticDetectionGate gate(config_);

    for (int i = 0; i < 30; ++i) {
        const double stamp = 0.1 * i;
        const auto result = gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp),
                                        camera_info_, field_, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u);
    }
    EXPECT_EQ(gate.last_stats().suppressed, 0);
}

TEST_F(StaticDetectionGateTest, PassesEverythingWithoutAFieldTransform) {
    StaticDetectionGate gate(config_);
    FieldDescription empty_field;
    empty_field.tf_camera_from_fieldcenter.tf = Eigen::Matrix4d::Zero();

    for (int i = 0; i < 30; ++i) {
        const double stamp = 0.1 * i;
        const auto result = gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp),
                                        camera_info_, empty_field, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u);
    }
}

TEST_F(StaticDetectionGateTest, CarriesBoxesThroughOnPassedAndSuppressedFrames) {
    StaticDetectionGate gate(config_);
    for (int i = 0; i < 11; ++i) {
        const double stamp = 0.1 * i;
        const auto result = gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp),
                                        camera_info_, field_, stamp);
        ASSERT_EQ(result.boxes.size(), 2u);
    }
    EXPECT_EQ(gate.last_stats().suppressed, 1);
}

TEST_F(StaticDetectionGateTest, PassesAGroupThatCannotBePlacedOnTheField) {
    StaticDetectionGate gate(config_);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (int i = 0; i < 30; ++i) {
        const double stamp = 0.1 * i;
        const auto result = gate.filter(make_pair(nan, 300.0, stamp), camera_info_, field_, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u);
    }
    EXPECT_EQ(gate.last_stats().suppressed, 0);
}

TEST_F(StaticDetectionGateTest, JudgesAnObservationAgainstTheClusterBeforeFoldingItIn) {
    config_.responsiveness_window = 1;
    StaticDetectionGate gate(config_);
    for (int i = 0; i < 11; ++i) {
        const double stamp = 0.1 * i;
        gate.filter(make_pair(kFloorGraphicPx.x, kFloorGraphicPx.y, stamp), camera_info_, field_,
                    stamp);
    }
    const auto arrival = gate.filter(make_pair(425.0, 300.0, 1.1), camera_info_, field_, 1.1);
    EXPECT_TRUE(arrival.keypoints.empty());
    const auto next = gate.filter(make_pair(425.0, 300.0, 1.2), camera_info_, field_, 1.2);
    EXPECT_EQ(next.keypoints.size(), 2u);
}

}  // namespace auto_battlebot
