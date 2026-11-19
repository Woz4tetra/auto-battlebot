#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>
#include <utility>

#include "remote/field_projection.hpp"

namespace auto_battlebot::remote {
namespace {

constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr double kFocalPx = 500.0;
constexpr double kCenterU = 320.0;
constexpr double kCenterV = 240.0;
constexpr double kFieldSizeX = 2.0;
constexpr double kFieldSizeY = 1.0;
constexpr double kFieldDepthM = 4.0;

CameraInfo make_camera() {
    CameraInfo info;
    info.width = kImageWidth;
    info.height = kImageHeight;
    info.intrinsics =
        (cv::Mat_<double>(3, 3) << kFocalPx, 0, kCenterU, 0, kFocalPx, kCenterV, 0, 0, 1);
    return info;
}

FieldDescription make_field(double depth) {
    FieldDescription field;
    field.size.size.x = kFieldSizeX;
    field.size.size.y = kFieldSizeY;
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 3) = depth;
    field.tf_camera_from_fieldcenter.tf = tf;
    return field;
}

TEST(FieldProjectionTest, WholeFieldIsOneClosedLoop) {
    constexpr size_t kSides = 4;
    constexpr size_t kSamplesPerSide = 4;
    const auto outline =
        project_field_outline(make_field(kFieldDepthM), make_camera(), kSamplesPerSide);
    ASSERT_EQ(outline.size(), 1u);
    const auto &loop = outline.front();
    ASSERT_EQ(loop.size(), kSides * kSamplesPerSide + 1) << "the first corner closes the loop";
    EXPECT_DOUBLE_EQ(loop.front().u, loop.back().u);
    EXPECT_DOUBLE_EQ(loop.front().v, loop.back().v);
    const double corner_x = -kFieldSizeX / 2.0;
    const double corner_y = -kFieldSizeY / 2.0;
    EXPECT_NEAR(loop.front().u, (kFocalPx * corner_x / kFieldDepthM + kCenterU) / kImageWidth,
                1e-9);
    EXPECT_NEAR(loop.front().v, (kFocalPx * corner_y / kFieldDepthM + kCenterV) / kImageHeight,
                1e-9);
}

FieldDescription field_tilted_behind_camera() {
    constexpr double kDepthPerFieldX = 1.0;
    constexpr double kDepthPerFieldZ = 0.0;
    FieldDescription field = make_field(0.0);
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 0) = kDepthPerFieldX;
    tf(2, 2) = kDepthPerFieldZ;
    field.tf_camera_from_fieldcenter.tf = tf;
    return field;
}

TEST(FieldProjectionTest, BorderBehindTheCameraIsCutNotDropped) {
    const FieldDescription field = field_tilted_behind_camera();
    const auto outline = project_field_outline(field, make_camera(), 8);
    ASSERT_EQ(outline.size(), 1u);
    EXPECT_GE(outline.front().size(), 2u);
}

TEST(FieldProjectionTest, ClippedLoopJoinsWrappedRun) {
    FieldDescription field = make_field(0.3);
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf(2, 1) = -1.0;
    tf(2, 2) = 0.0;
    tf(2, 3) = 0.3;
    field.tf_camera_from_fieldcenter.tf = tf;
    const auto outline = project_field_outline(field, make_camera(), 8);
    ASSERT_EQ(outline.size(), 1u) << "a loop cut behind the camera draws one line";
    EXPECT_NE(outline.front().front().u, outline.front().back().u);
}

TEST(FieldProjectionTest, ProjectsASinglePoint) {
    const auto center = project_field_point(make_field(kFieldDepthM), make_camera(), 0.0, 0.0);
    ASSERT_TRUE(center.has_value());
    EXPECT_NEAR(center->u, 0.5, 1e-9);
    EXPECT_NEAR(center->v, 0.5, 1e-9);
    EXPECT_FALSE(project_field_point(make_field(-1.0), make_camera(), 0.0, 0.0).has_value());
}

TEST(FieldProjectionTest, UnprojectInvertsProject) {
    constexpr double kCameraHeightM = 1.2;
    constexpr double kPitchDownDeg = 35.0;
    constexpr double kForwardToYDeg = 90.0;
    FieldDescription field = make_field(0.0);
    Eigen::Matrix4d camera_from_field = Eigen::Matrix4d::Identity();
    const double pitch = -(kForwardToYDeg + kPitchDownDeg) * M_PI / 180.0;
    camera_from_field.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()).toRotationMatrix().transpose();
    const Eigen::Vector3d camera_in_field(0.3, -2.0, kCameraHeightM);
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
    const auto ahead = unproject_image_point(make_field(kFieldDepthM), make_camera(), center);
    ASSERT_TRUE(ahead.has_value());
    EXPECT_NEAR(ahead->x, 0.0, 1e-9);
    EXPECT_NEAR(ahead->y, 0.0, 1e-9);
    EXPECT_FALSE(unproject_image_point(make_field(-1.0), make_camera(), center).has_value());
}

struct SvelteTriangle {
    double view = 0.0;
    std::array<std::array<double, 2>, 3> corners{};
};

SvelteTriangle read_top_down_triangle() {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
                      "web" / "src" / "components" / "TopDown.svelte";
    std::ifstream in(path);
    EXPECT_TRUE(in.good()) << path;
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    SvelteTriangle triangle;
    std::smatch view;
    EXPECT_TRUE(std::regex_search(text, view, std::regex(R"(const VIEW = (\d+);)")));
    triangle.view = std::stod(view[1]);
    std::smatch cam;
    const std::regex pattern(R"(class="cam" d="M (\S+) (\S+) L (\S+) (\S+) L (\S+) (\S+) Z")");
    EXPECT_TRUE(std::regex_search(text, cam, pattern));
    for (size_t i = 0; i < 3; ++i) {
        triangle.corners[i] = {std::stod(cam[1 + 2 * i]) - triangle.view / 2.0,
                               std::stod(cam[2 + 2 * i]) - triangle.view / 2.0};
    }
    return triangle;
}

TEST(FieldProjectionTest, CameraMarkerMatchesTheTopDownTriangle) {
    const SvelteTriangle triangle = read_top_down_triangle();
    const auto marker = project_camera_marker(make_field(kFieldDepthM), make_camera());
    ASSERT_EQ(marker.size(), 3u);
    constexpr double kLongestSideM = kFieldSizeX;
    constexpr double kBoxPx = 192.0;
    const double m_per_px = kLongestSideM / kBoxPx;
    for (size_t i = 0; i < 3; ++i) {
        const double field_x = triangle.corners[i][0] * m_per_px;
        const double field_y = -triangle.corners[i][1] * m_per_px;
        EXPECT_NEAR(marker[i].u, (kFocalPx * field_x / kFieldDepthM + kCenterU) / kImageWidth, 1e-9)
            << "corner " << i;
        EXPECT_NEAR(marker[i].v, (kFocalPx * field_y / kFieldDepthM + kCenterV) / kImageHeight,
                    1e-9)
            << "corner " << i;
    }
    EXPECT_TRUE(project_camera_marker(make_field(-1.0), make_camera()).empty());
}

TEST(FieldProjectionTest, MissingInputsGiveNoOutline) {
    EXPECT_TRUE(project_field_outline(make_field(kFieldDepthM), CameraInfo{}).empty());
    FieldDescription empty;
    EXPECT_TRUE(project_field_outline(empty, make_camera()).empty());
}

}  // namespace
}  // namespace auto_battlebot::remote
