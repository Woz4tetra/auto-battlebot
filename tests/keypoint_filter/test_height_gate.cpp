#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

#include "keypoint_filter/height_gate.hpp"

namespace auto_battlebot {

/**
 * The field transform puts the plane 3 m down the optical axis with the field z axis flipped, so
 * the camera sits 3 m above it. The plane normal ends up parallel to the optical axis, which makes
 * a keypoint's height exactly (3.0 - depth) at every pixel and keeps the arithmetic in these tests
 * obvious.
 */
class KeypointHeightGateTest : public ::testing::Test {
   protected:
    static constexpr double kCameraHeight = 3.0;

    KeypointHeightGateConfiguration config_;
    CameraInfo camera_info_;
    FieldDescription field_;

    void SetUp() override {
        config_.reject_enable = true;
        config_.min_elevation_meters = 0.015;
        config_.max_height_meters = 0.40;
        config_.sample_radius_px = 3;
        config_.min_valid_samples = 8;
        config_.max_circle_samples = 1024;
        config_.top_percentile = 0.90;
        config_.floor_percentile = 0.25;
        config_.ring_inner_scale = 1.4;
        config_.ring_outer_scale = 2.4;

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
        tf(2, 3) = kCameraHeight;
        field_.tf_camera_from_fieldcenter.tf = tf;
    }

    /** A front/back pair sharing one detection_index, plus the box they came from, as the blob
     *  model emits them. The box is the keypoints' bounding rectangle inflated by `pad`, so its
     *  inscribed circle covers the midline the way a real detection box covers a robot. */
    static ModelResultStamped make_pair(double x_a, double y_a, double x_b, double y_b,
                                        double pad = 20.0) {
        ModelResultStamped keypoints;
        keypoints.header.frame_id = FrameId::CAMERA;
        keypoints.header.stamp = 1.0;

        Keypoint a;
        a.label = Label::OPPONENT;
        a.keypoint_label = KeypointLabel::OPPONENT_FRONT;
        a.x = x_a;
        a.y = y_a;
        a.confidence = 0.9;
        a.detection_index = 1;

        Keypoint b = a;
        b.keypoint_label = KeypointLabel::OPPONENT_BACK;
        b.x = x_b;
        b.y = y_b;

        keypoints.keypoints = {a, b};
        keypoints.boxes.resize(2);
        keypoints.boxes[1] = BoundingBox{std::min(x_a, x_b) - pad, std::min(y_a, y_b) - pad,
                                         std::max(x_a, x_b) + pad, std::max(y_a, y_b) + pad};
        return keypoints;
    }

    /** Flat floor across the whole image, at zero height above the plane. */
    DepthImage make_floor() const {
        DepthImage image;
        image.header.frame_id = FrameId::CAMERA;
        image.image =
            cv::Mat(camera_info_.height, camera_info_.width, CV_32FC1, cv::Scalar(kCameraHeight));
        return image;
    }

    /** Raise a disc `height` metres above whatever it is drawn on. */
    static void raise_disc(DepthImage &depth, int x, int y, int radius, double height) {
        cv::circle(depth.image, cv::Point(x, y), radius, cv::Scalar(kCameraHeight - height),
                   cv::FILLED);
    }
};

TEST_F(KeypointHeightGateTest, RejectsDetectionFlatOnTheField) {
    // Nothing stands proud of its surroundings: a painted logo.
    KeypointHeightGate gate(config_);
    const auto result =
        gate.filter(make_pair(300.0, 240.0, 340.0, 240.0), make_floor(), camera_info_, field_);

    EXPECT_TRUE(result.keypoints.empty());
    EXPECT_EQ(gate.last_stats().rejected_low, 1);
    EXPECT_EQ(gate.last_stats().groups_passed, 0);
}

TEST_F(KeypointHeightGateTest, RejectsAFlatDetectionOnARaisedFloor) {
    // The point of measuring relief instead of absolute height: if the fitted plane has drifted,
    // or the whole area sits above it, a flat graphic there is still flat and must still go.
    KeypointHeightGate gate(config_);
    DepthImage depth = make_floor();
    depth.image.setTo(kCameraHeight - 0.08);  // everything 80 mm "above" the plane

    const auto result =
        gate.filter(make_pair(300.0, 240.0, 340.0, 240.0), depth, camera_info_, field_);

    EXPECT_TRUE(result.keypoints.empty()) << "absolute height would have passed this";
    EXPECT_EQ(gate.last_stats().rejected_low, 1);
}

TEST_F(KeypointHeightGateTest, KeepsRobotStandingProudOfTheField) {
    // A 50 mm disc that fills the inscribed circle but leaves the floor ring clear.
    KeypointHeightGate gate(config_);
    DepthImage depth = make_floor();
    raise_disc(depth, 320, 240, 35, 0.05);

    const auto result =
        gate.filter(make_pair(305.0, 240.0, 335.0, 240.0, 30.0), depth, camera_info_, field_);

    ASSERT_EQ(result.keypoints.size(), 2u);
    EXPECT_EQ(gate.last_stats().groups_passed, 1);
    EXPECT_EQ(gate.last_stats().rejected_low, 0);
    for (const Keypoint &kp : result.keypoints) {
        EXPECT_NEAR(kp.height_above_plane, 0.05, 1e-6);
    }
}

TEST_F(KeypointHeightGateTest, RejectsDetectionAboveTheBand) {
    // A metre off the floor is a wall or a person leaning in, not a robot. Relief alone would
    // welcome it, so the ceiling is judged on absolute height above the plane.
    KeypointHeightGate gate(config_);
    DepthImage depth = make_floor();
    raise_disc(depth, 320, 240, 35, 1.0);

    const auto result =
        gate.filter(make_pair(305.0, 240.0, 335.0, 240.0, 30.0), depth, camera_info_, field_);

    EXPECT_TRUE(result.keypoints.empty());
    EXPECT_EQ(gate.last_stats().rejected_high, 1);
}

TEST_F(KeypointHeightGateTest, AbstainsWithoutDepth) {
    // A dropped depth frame must never blind the pipeline.
    KeypointHeightGate gate(config_);
    const auto input = make_pair(300.0, 240.0, 340.0, 240.0);
    const auto result = gate.filter(input, DepthImage{}, camera_info_, field_);

    ASSERT_EQ(result.keypoints.size(), 2u);
    for (const Keypoint &kp : result.keypoints) {
        EXPECT_FALSE(std::isfinite(kp.height_above_plane));
    }
}

TEST_F(KeypointHeightGateTest, AbstainsWhenDepthIsAllInvalid) {
    KeypointHeightGate gate(config_);
    DepthImage depth = make_floor();
    depth.image.setTo(std::numeric_limits<float>::quiet_NaN());

    const auto result =
        gate.filter(make_pair(300.0, 220.0, 340.0, 260.0), depth, camera_info_, field_);

    ASSERT_EQ(result.keypoints.size(), 2u);
    EXPECT_EQ(gate.last_stats().abstained, 1);
    EXPECT_EQ(gate.last_stats().rejected_low, 0);
}

TEST_F(KeypointHeightGateTest, AbstainsWithoutAFieldTransform) {
    KeypointHeightGate gate(config_);
    FieldDescription empty_field;
    const auto result =
        gate.filter(make_pair(300.0, 240.0, 340.0, 240.0), make_floor(), camera_info_, empty_field);

    EXPECT_EQ(result.keypoints.size(), 2u);
}

TEST_F(KeypointHeightGateTest, RejectionDisabledPassesEverythingButStillMeasures) {
    config_.reject_enable = false;
    KeypointHeightGate gate(config_);
    const auto result =
        gate.filter(make_pair(300.0, 240.0, 340.0, 240.0), make_floor(), camera_info_, field_);

    // Flat on the field, but rejection is off, so it survives carrying its measured height.
    ASSERT_EQ(result.keypoints.size(), 2u);
    for (const Keypoint &kp : result.keypoints) {
        EXPECT_NEAR(kp.height_above_plane, 0.0, 1e-6);
    }
}

TEST_F(KeypointHeightGateTest, MeasuresHeightInsideTheBoxNotAtTheKeypoints) {
    // The real failure this gate had to solve: blob keypoints sit at the ends of a box midline,
    // which land on the floor beside a robot. Floor everywhere except a raised disc over the box
    // centre. Sampling at the keypoints reads 0; sampling the inscribed circle reads the robot.
    config_.reject_enable = true;
    KeypointHeightGate gate(config_);

    DepthImage depth = make_floor();
    raise_disc(depth, 320, 240, 10, 0.05);

    // Keypoints at the box's midline ends, both over bare floor.
    const auto result =
        gate.filter(make_pair(280.0, 240.0, 360.0, 240.0, 8.0), depth, camera_info_, field_);

    ASSERT_EQ(result.keypoints.size(), 2u) << "region median read the floor, not the robot";
    EXPECT_EQ(gate.last_stats().rejected_low, 0);
}

TEST_F(KeypointHeightGateTest, AbstainsWithoutABox) {
    // A model that produces no boxes cannot be gated, only annotated.
    config_.reject_enable = true;
    KeypointHeightGate gate(config_);
    auto input = make_pair(300.0, 240.0, 340.0, 240.0);
    input.boxes.clear();

    const auto result = gate.filter(input, make_floor(), camera_info_, field_);

    ASSERT_EQ(result.keypoints.size(), 2u);
    EXPECT_EQ(gate.last_stats().abstained, 1);
    EXPECT_EQ(gate.last_stats().rejected_low, 0);
    for (const Keypoint &kp : result.keypoints) {
        EXPECT_NEAR(kp.height_above_plane, 0.0, 1e-6);
    }
}

TEST_F(KeypointHeightGateTest, GroupsAreKeptOrDroppedWhole) {
    // One keypoint of the pair sits over a hole in the depth map. The group still measures fine
    // from its other member, and both keypoints survive carrying the group height, because a
    // half-dropped front/back pair is useless downstream.
    KeypointHeightGate gate(config_);
    DepthImage depth = make_floor();
    raise_disc(depth, 320, 240, 35, 0.05);
    cv::circle(depth.image, cv::Point(305, 240), 6,
               cv::Scalar(std::numeric_limits<float>::quiet_NaN()), cv::FILLED);

    const auto result =
        gate.filter(make_pair(305.0, 240.0, 335.0, 240.0, 30.0), depth, camera_info_, field_);

    ASSERT_EQ(result.keypoints.size(), 2u);
    for (const Keypoint &kp : result.keypoints) {
        EXPECT_NEAR(kp.height_above_plane, 0.05, 1e-6);
    }
}

}  // namespace auto_battlebot
