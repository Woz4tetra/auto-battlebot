#include <gtest/gtest.h>

#include <cmath>
#include <utility>

#include "remote/field_projection.hpp"

namespace auto_battlebot::remote {
namespace {

CameraInfo make_camera() {
    CameraInfo info;
    info.width = 640;
    info.height = 480;
    info.intrinsics = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);
    return info;
}

/** A 2 x 1 m field `depth` meters straight ahead, facing the camera. */
FieldDescription make_field(double depth) {
    FieldDescription field;
    field.size.size.x = 2.0;
    field.size.size.y = 1.0;
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 3) = depth;
    field.tf_camera_from_fieldcenter.tf = tf;
    return field;
}

TEST(FieldProjectionTest, WholeFieldIsOneClosedLoop) {
    const auto outline = project_field_outline(make_field(4.0), make_camera(), 4);
    ASSERT_EQ(outline.size(), 1u);
    const auto &loop = outline.front();
    ASSERT_EQ(loop.size(), 17u);  // 4 sides x 4 samples, plus the first corner again.
    EXPECT_DOUBLE_EQ(loop.front().u, loop.back().u);
    EXPECT_DOUBLE_EQ(loop.front().v, loop.back().v);
    // Corner (-1, -0.5) at 4 m: u = (500 * -0.25 + 320) / 640.
    EXPECT_NEAR(loop.front().u, 195.0 / 640.0, 1e-9);
    EXPECT_NEAR(loop.front().v, 177.5 / 480.0, 1e-9);
}

TEST(FieldProjectionTest, BorderBehindTheCameraIsCutNotDropped) {
    // Tilt the field so its far half is behind the camera plane.
    FieldDescription field = make_field(0.0);
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 0) = 1.0;  // depth grows with field x, so x < 0.05 m is behind or too close
    tf(2, 2) = 0.0;
    field.tf_camera_from_fieldcenter.tf = tf;
    const auto outline = project_field_outline(field, make_camera(), 8);
    ASSERT_EQ(outline.size(), 1u);
    EXPECT_GE(outline.front().size(), 2u);
}

TEST(FieldProjectionTest, ProjectsASinglePoint) {
    const auto center = project_field_point(make_field(4.0), make_camera(), 0.0, 0.0);
    ASSERT_TRUE(center.has_value());
    EXPECT_NEAR(center->u, 0.5, 1e-9);
    EXPECT_NEAR(center->v, 0.5, 1e-9);
    EXPECT_FALSE(project_field_point(make_field(-1.0), make_camera(), 0.0, 0.0).has_value());
}

TEST(FieldProjectionTest, UnprojectInvertsProject) {
    // A camera 1.2 m up, pitched 35 degrees down at the floor, shifted off center.
    FieldDescription field = make_field(0.0);
    Eigen::Matrix4d camera_from_field = Eigen::Matrix4d::Identity();
    // Rotation about field x that turns the camera's +z (forward) to +y, then 35 degrees down.
    const double pitch = -(90.0 + 35.0) * M_PI / 180.0;
    camera_from_field.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()).toRotationMatrix().transpose();
    const Eigen::Vector3d camera_in_field(0.3, -2.0, 1.2);
    camera_from_field.topRightCorner<3, 1>() =
        -camera_from_field.topLeftCorner<3, 3>() * camera_in_field;
    field.tf_camera_from_fieldcenter.tf = camera_from_field;

    for (const auto &[x, y] : {std::pair{0.0, 0.0}, {0.8, 0.4}, {-0.9, -0.45}}) {
        const auto image = project_field_point(field, make_camera(), x, y);
        ASSERT_TRUE(image.has_value());
        const auto back = unproject_image_point(field, make_camera(), *image);
        ASSERT_TRUE(back.has_value());
        EXPECT_NEAR(back->x, x, 1e-9);
        EXPECT_NEAR(back->y, y, 1e-9);
    }
}

TEST(FieldProjectionTest, UnprojectRefusesAFloorBehindTheCamera) {
    const ImagePoint center{.u = 0.5, .v = 0.5};
    const auto ahead = unproject_image_point(make_field(4.0), make_camera(), center);
    ASSERT_TRUE(ahead.has_value());
    EXPECT_NEAR(ahead->x, 0.0, 1e-9);
    EXPECT_NEAR(ahead->y, 0.0, 1e-9);
    EXPECT_FALSE(unproject_image_point(make_field(-1.0), make_camera(), center).has_value());
}

TEST(FieldProjectionTest, CameraMarkerMatchesTheTopDownTriangle) {
    // Longest side 2 m over the 192 px box: tip at y = -116 px * 2 / 192 m, base 8 px either side.
    const auto marker = project_camera_marker(make_field(4.0), make_camera());
    ASSERT_EQ(marker.size(), 3u);
    const double m_per_px = 2.0 / 192.0;
    EXPECT_NEAR(marker[0].u, 0.5, 1e-9);
    EXPECT_NEAR(marker[0].v, (500.0 * -116.0 * m_per_px / 4.0 + 240.0) / 480.0, 1e-9);
    EXPECT_NEAR(marker[1].u, (500.0 * -8.0 * m_per_px / 4.0 + 320.0) / 640.0, 1e-9);
    EXPECT_NEAR(marker[2].u, 1.0 - marker[1].u, 1e-9);
    EXPECT_NEAR(marker[1].v, marker[2].v, 1e-9);
    EXPECT_TRUE(project_camera_marker(make_field(-1.0), make_camera()).empty());
}

TEST(FieldProjectionTest, MissingInputsGiveNoOutline) {
    EXPECT_TRUE(project_field_outline(make_field(4.0), CameraInfo{}).empty());
    FieldDescription empty;
    EXPECT_TRUE(project_field_outline(empty, make_camera()).empty());
}

}  // namespace
}  // namespace auto_battlebot::remote
