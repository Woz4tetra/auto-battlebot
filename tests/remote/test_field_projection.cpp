#include <gtest/gtest.h>

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

TEST(FieldProjectionTest, MissingInputsGiveNoOutline) {
    EXPECT_TRUE(project_field_outline(make_field(4.0), CameraInfo{}).empty());
    FieldDescription empty;
    EXPECT_TRUE(project_field_outline(empty, make_camera()).empty());
}

}  // namespace
}  // namespace auto_battlebot::remote
