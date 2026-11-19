#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "field_filter/field_outline.hpp"
#include "field_filter/field_pose.hpp"
#include "field_filter/synthetic_field.hpp"

namespace auto_battlebot {
namespace {
constexpr double kFieldSize = 2.35;
const cv::Size kImageSize(1920, 1200);

FieldOutline outline_for(const Eigen::Matrix4d &tf_camera_from_fieldcenter) {
    const cv::Mat mask = testing_support::render_field_mask(tf_camera_from_fieldcenter,
                                                            testing_support::test_intrinsics(),
                                                            kImageSize, kFieldSize, kFieldSize);
    return extract_field_outline(find_largest_contour_mask(mask), FieldOutlineParams{});
}

double shoelace_area(const std::array<cv::Point2d, 4> &quad) {
    double total = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        const cv::Point2d &a = quad[i];
        const cv::Point2d &b = quad[(i + 1) % 4];
        total += a.x * b.y - b.x * a.y;
    }
    return total / 2.0;
}

double distance_to_line_px(const cv::Vec3d &line, const cv::Point2d &point) {
    return std::abs(line[0] * point.x + line[1] * point.y + line[2]) / std::hypot(line[0], line[1]);
}
}  // namespace

TEST(FieldOutlineTest, CornersWindFromMinXPlusYByIncreasingAngle) {
    const FieldOutline outline = outline_for(testing_support::camera_pose(3.0, 5.0, 12.0));
    ASSERT_TRUE(outline.ok) << outline.failure;
    ASSERT_EQ(outline.supported_sides, 4);

    const auto sum = [](const cv::Point2d &corner) { return corner.x + corner.y; };
    for (size_t i = 1; i < 4; ++i) {
        EXPECT_LT(sum(outline.corners[0]), sum(outline.corners[i])) << "corner " << i;
    }
    EXPECT_GT(shoelace_area(outline.corners), 0.0) << "winding";
}

TEST(FieldOutlineTest, LinesFollowCornerOrder) {
    const FieldOutline outline = outline_for(testing_support::camera_pose(3.0, 5.0, 12.0));
    ASSERT_TRUE(outline.ok) << outline.failure;
    ASSERT_EQ(outline.supported_sides, 4);

    for (size_t side = 0; side < 4; ++side) {
        EXPECT_LT(distance_to_line_px(outline.lines[side], outline.corners[side]), 1.0)
            << "side " << side << " start corner";
        EXPECT_LT(distance_to_line_px(outline.lines[side], outline.corners[(side + 1) % 4]), 1.0)
            << "side " << side << " end corner";
    }
}

TEST(FieldOutlineTest, ClippedSideIsReportedUnsupportedNotRejected) {
    const FieldOutline outline = outline_for(testing_support::camera_pose(1.6, 55.0, 0.0));

    ASSERT_TRUE(outline.ok) << outline.failure;
    EXPECT_EQ(outline.supported_sides, 3);
    ASSERT_EQ(std::count(outline.side_supported.begin(), outline.side_supported.end(), false), 1);
    for (size_t side = 0; side < 4; ++side) {
        EXPECT_EQ(outline.side_rms_px[side].has_value(), outline.side_supported[side])
            << "side " << side;
    }
}

TEST(FieldPoseTest, ThreeLineSolveKeepsWindingNotMirror) {
    const Eigen::Matrix3d intrinsics = testing_support::test_intrinsics();
    int solved = 0;
    for (const double yaw : {0.0, 20.0, -30.0, 90.0, 180.0}) {
        for (const double tilt : {50.0, 55.0, 60.0}) {
            const FieldOutline outline = outline_for(testing_support::camera_pose(1.6, tilt, yaw));
            if (!outline.ok || outline.supported_sides != 3) {
                continue;
            }
            const std::array<cv::Point2d, 4> mirrored = {outline.corners[0], outline.corners[3],
                                                         outline.corners[2], outline.corners[1]};
            for (const auto &observed : {outline.corners, mirrored}) {
                const FieldPoseResult pose =
                    pose_from_three_lines(outline.lines, outline.side_supported, observed,
                                          kFieldSize, kFieldSize, intrinsics);
                if (!pose.ok) {
                    continue;
                }
                ++solved;
                const auto projected = project_field_corners(pose.tf_camera_from_fieldcenter,
                                                             kFieldSize, kFieldSize, intrinsics);
                EXPECT_GT(shoelace_area(projected) * shoelace_area(observed), 0.0)
                    << "yaw " << yaw << " tilt " << tilt;
            }
        }
    }
    EXPECT_GT(solved, 0);
}

TEST(FieldPoseTest, ThreeLineSolveRecoversThePoseWhenWindingMatches) {
    const FieldOutline outline = outline_for(testing_support::camera_pose(1.6, 55.0, 0.0));
    ASSERT_EQ(outline.supported_sides, 3);
    const FieldPoseResult pose =
        pose_from_three_lines(outline.lines, outline.side_supported, outline.corners, kFieldSize,
                              kFieldSize, testing_support::test_intrinsics());
    EXPECT_TRUE(pose.ok) << pose.failure;
}
}  // namespace auto_battlebot
