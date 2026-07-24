#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "transform_utils.hpp"

namespace auto_battlebot {

class PlaneHeightOffsetTest : public ::testing::Test {
   protected:
    // Camera at origin looking down at a plane 3 m below (camera frame, z forward).
    Eigen::Vector3d plane_center_{0.0, 0.0, 3.0};
    Eigen::Vector3d plane_normal_{0.0, 0.0, -1.0};

    CameraInfo camera_info_;

    void SetUp() override {
        camera_info_.width = 640;
        camera_info_.height = 480;
        camera_info_.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        camera_info_.intrinsics.at<double>(0, 0) = 500.0;
        camera_info_.intrinsics.at<double>(1, 1) = 500.0;
        camera_info_.intrinsics.at<double>(0, 2) = 320.0;
        camera_info_.intrinsics.at<double>(1, 2) = 240.0;
    }
};

TEST_F(PlaneHeightOffsetTest, ShiftsInwardTowardNadir) {
    const Eigen::Vector3d plane_point(1.0, 0.5, 3.0);
    const double keypoint_height = 0.06;

    const Eigen::Vector3d corrected =
        correct_plane_height_offset(plane_point, plane_center_, plane_normal_, keypoint_height);

    // Camera height is 3.0, so the point contracts toward the nadir (0, 0, 3) by (3 - 0.06) / 3.
    const double scale = (3.0 - keypoint_height) / 3.0;
    EXPECT_NEAR(corrected.x(), 1.0 * scale, 1e-12);
    EXPECT_NEAR(corrected.y(), 0.5 * scale, 1e-12);
    EXPECT_NEAR(corrected.z(), 3.0, 1e-12);
}

TEST_F(PlaneHeightOffsetTest, ExactlyInvertsForwardProjectionOnTiltedPlane) {
    const Eigen::Vector3d plane_center(0.3, -0.2, 3.0);
    const Eigen::Vector3d plane_normal = Eigen::Vector3d(0.2, -0.1, -1.0).normalized();
    const double keypoint_height = 0.06;

    // Build a ground point on the plane, then the physical keypoint sitting above it.
    const Eigen::Vector3d u = plane_normal.unitOrthogonal();
    const Eigen::Vector3d v = plane_normal.cross(u);
    const Eigen::Vector3d ground_point = plane_center + 0.8 * u - 0.4 * v;
    const Eigen::Vector3d physical_keypoint = ground_point + keypoint_height * plane_normal;

    // Forward-project the physical keypoint onto the plane along the camera ray.
    const double t = plane_normal.dot(plane_center) / plane_normal.dot(physical_keypoint);
    const Eigen::Vector3d plane_point = t * physical_keypoint;

    const Eigen::Vector3d corrected =
        correct_plane_height_offset(plane_point, plane_center, plane_normal, keypoint_height);

    EXPECT_NEAR(corrected.x(), ground_point.x(), 1e-9);
    EXPECT_NEAR(corrected.y(), ground_point.y(), 1e-9);
    EXPECT_NEAR(corrected.z(), ground_point.z(), 1e-9);
}

TEST_F(PlaneHeightOffsetTest, ZeroHeightIsNoOp) {
    const Eigen::Vector3d plane_point(1.0, 0.5, 3.0);
    const Eigen::Vector3d corrected =
        correct_plane_height_offset(plane_point, plane_center_, plane_normal_, 0.0);
    EXPECT_EQ(corrected, plane_point);
}

TEST_F(PlaneHeightOffsetTest, DegenerateCameraAtPlaneIsNoOp) {
    // Plane passes through the camera origin: camera height is zero.
    const Eigen::Vector3d plane_point(1.0, 0.5, 0.0);
    const Eigen::Vector3d corrected =
        correct_plane_height_offset(plane_point, Eigen::Vector3d::Zero(), plane_normal_, 0.06);
    EXPECT_EQ(corrected, plane_point);
}

TEST_F(PlaneHeightOffsetTest, KeypointAboveCameraIsNoOp) {
    const Eigen::Vector3d plane_point(1.0, 0.5, 3.0);
    const Eigen::Vector3d corrected =
        correct_plane_height_offset(plane_point, plane_center_, plane_normal_, 3.5);
    EXPECT_EQ(corrected, plane_point);
}

TEST_F(PlaneHeightOffsetTest, ProjectionOverloadMatchesBaseThenCorrects) {
    Keypoint keypoint;
    keypoint.x = 420.0;
    keypoint.y = 300.0;

    Eigen::Vector3d uncorrected;
    ASSERT_TRUE(project_keypoint_onto_plane(keypoint, plane_center_, plane_normal_, camera_info_,
                                            uncorrected));

    Eigen::Vector3d with_zero_height;
    ASSERT_TRUE(project_keypoint_onto_plane(keypoint, plane_center_, plane_normal_, camera_info_,
                                            0.0, with_zero_height));
    EXPECT_EQ(with_zero_height, uncorrected);

    Eigen::Vector3d corrected;
    ASSERT_TRUE(project_keypoint_onto_plane(keypoint, plane_center_, plane_normal_, camera_info_,
                                            0.06, corrected));

    // The corrected point moves inward: closer to the nadir than the uncorrected projection.
    const Eigen::Vector3d nadir(0.0, 0.0, 3.0);
    EXPECT_LT((corrected - nadir).norm(), (uncorrected - nadir).norm());
    EXPECT_EQ(corrected,
              correct_plane_height_offset(uncorrected, plane_center_, plane_normal_, 0.06));
}

}  // namespace auto_battlebot
