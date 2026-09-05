#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "keypoint_filter/static_gate.hpp"

namespace auto_battlebot {

/** Same 3 m overhead field geometry as the height gate tests, so a pixel maps to a field position
 *  by a fixed scale and "move the detection" means "move the pixel". */
class StaticDetectionGateTest : public ::testing::Test {
   protected:
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
        camera_info_.intrinsics.at<double>(0, 0) = 500.0;
        camera_info_.intrinsics.at<double>(1, 1) = 500.0;
        camera_info_.intrinsics.at<double>(0, 2) = 320.0;
        camera_info_.intrinsics.at<double>(1, 2) = 240.0;

        field_.child_frame_id = FrameId::FIELD;
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf(2, 2) = -1.0;
        tf(2, 3) = 3.0;
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

    // Ten frames over one second at a fixed pixel: an arena floor graphic.
    for (int i = 0; i < 10; ++i) {
        const double stamp = 0.1 * i;
        const auto result =
            gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, field_, stamp);
        EXPECT_EQ(result.keypoints.size(), 2u) << "suppressed too early at stamp " << stamp;
    }

    // Past the dwell, with the observation count met, it goes.
    const auto result = gate.filter(make_pair(400.0, 300.0, 1.0), camera_info_, field_, 1.0);
    EXPECT_TRUE(result.keypoints.empty());
    EXPECT_EQ(gate.last_stats().suppressed, 1);
    EXPECT_EQ(gate.last_stats().static_clusters, 1);
}

TEST_F(StaticDetectionGateTest, NeverSuppressesAMovingDetection) {
    StaticDetectionGate gate(config_);

    // Drift 4 px per frame. At 3 m and fx 500 that is 24 mm per frame, so the detection walks
    // clear of static_radius quickly and out of match_radius soon after.
    for (int i = 0; i < 40; ++i) {
        const double stamp = 0.1 * i;
        const auto result =
            gate.filter(make_pair(200.0 + 4.0 * i, 300.0, stamp), camera_info_, field_, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u)
            << "suppressed a moving detection at stamp " << stamp;
    }
    EXPECT_EQ(gate.last_stats().suppressed, 0);
}

TEST_F(StaticDetectionGateTest, ARobotMovingInsideTheMatchRadiusIsNotScenery) {
    StaticDetectionGate gate(config_);

    // Working back and forth inside the match radius but well past the static radius: a robot
    // nudging around in one corner, not a painted graphic. At 3 m and fx 500, 25 px is 150 mm,
    // so the mean deviation settles near 75 mm against a 50 mm static radius.
    for (int i = 0; i < 60; ++i) {
        const double stamp = 0.1 * i;
        const double x = (i % 2 == 0) ? 400.0 : 425.0;
        const auto result = gate.filter(make_pair(x, 300.0, stamp), camera_info_, field_, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u) << "suppressed a moving robot at stamp " << stamp;
    }
    EXPECT_EQ(gate.last_stats().suppressed, 0);
}

TEST_F(StaticDetectionGateTest, ARobotArrivingOnALogoLiftsSuppressionThenItReturns) {
    // The failure direction that matters. Once a graphic is suppressed, a robot driving onto it
    // must not stay invisible: its motion widens the cluster and switches suppression off. When it
    // leaves, the graphic goes back to being suppressed without needing a restart.
    StaticDetectionGate gate(config_);

    double stamp = 0.0;
    for (int i = 0; i < 60; ++i, stamp += 0.1) {
        gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, field_, stamp);
    }
    ASSERT_TRUE(
        gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, field_, stamp).keypoints.empty())
        << "the graphic should be suppressed by now";
    stamp += 0.1;

    // A robot arrives and works across the cluster.
    bool lifted = false;
    for (int i = 0; i < 40; ++i, stamp += 0.1) {
        const double x = (i % 2 == 0) ? 385.0 : 430.0;
        if (!gate.filter(make_pair(x, 300.0, stamp), camera_info_, field_, stamp)
                 .keypoints.empty()) {
            lifted = true;
        }
    }
    EXPECT_TRUE(lifted) << "a robot on a suppressed cluster stayed invisible";

    // It leaves; the graphic alone settles back down and suppression resumes.
    bool resuppressed = false;
    for (int i = 0; i < 120; ++i, stamp += 0.1) {
        if (gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, field_, stamp)
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
        gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, field_, stamp);
    }
    ASSERT_TRUE(
        gate.filter(make_pair(400.0, 300.0, 1.5), camera_info_, field_, 1.5).keypoints.empty());

    // Nothing seen for longer than forget_seconds, so the cluster is dropped and the next
    // detection at that spot starts a fresh dwell instead of being suppressed on arrival.
    const auto result = gate.filter(make_pair(400.0, 300.0, 10.0), camera_info_, field_, 10.0);
    EXPECT_EQ(result.keypoints.size(), 2u);
}

TEST_F(StaticDetectionGateTest, ResetClearsClustersOnFieldReinit) {
    StaticDetectionGate gate(config_);

    for (int i = 0; i < 15; ++i) {
        const double stamp = 0.1 * i;
        gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, field_, stamp);
    }
    ASSERT_TRUE(
        gate.filter(make_pair(400.0, 300.0, 1.5), camera_info_, field_, 1.5).keypoints.empty());

    // Field coordinates recorded against the old origin mean nothing after a re-init.
    gate.reset();
    const auto result = gate.filter(make_pair(400.0, 300.0, 1.6), camera_info_, field_, 1.6);
    EXPECT_EQ(result.keypoints.size(), 2u);
}

TEST_F(StaticDetectionGateTest, DisabledGatePassesEverything) {
    config_.enable = false;
    StaticDetectionGate gate(config_);

    for (int i = 0; i < 30; ++i) {
        const double stamp = 0.1 * i;
        const auto result =
            gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, field_, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u);
    }
    EXPECT_EQ(gate.last_stats().suppressed, 0);
}

TEST_F(StaticDetectionGateTest, PassesEverythingWithoutAFieldTransform) {
    StaticDetectionGate gate(config_);
    FieldDescription empty_field;

    for (int i = 0; i < 30; ++i) {
        const double stamp = 0.1 * i;
        const auto result =
            gate.filter(make_pair(400.0, 300.0, stamp), camera_info_, empty_field, stamp);
        ASSERT_EQ(result.keypoints.size(), 2u);
    }
}

}  // namespace auto_battlebot
