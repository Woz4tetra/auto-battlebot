#include <gtest/gtest.h>

#include "field_filter/config.hpp"
#include "field_filter/field_outline.hpp"
#include "field_filter/field_pose.hpp"
#include "field_filter/homography_field_filter.hpp"
#include "field_filter/synthetic_field.hpp"

namespace auto_battlebot {
namespace {
constexpr double kFieldSize = 2.35;
const cv::Size kImageSize(1920, 1200);

HomographyFieldFilterConfiguration make_config() {
    HomographyFieldFilterConfiguration config;
    config.field_size_x = kFieldSize;
    config.field_size_y = kFieldSize;
    return config;
}

CameraData make_camera_data() {
    CameraData camera_data;
    camera_data.camera_info.width = kImageSize.width;
    camera_data.camera_info.height = kImageSize.height;
    camera_data.camera_info.intrinsics = testing_support::test_intrinsics_cv();
    camera_data.rgb.header.stamp = 12.5;
    return camera_data;
}

MaskStamped make_mask(const cv::Mat &mask) {
    MaskStamped stamped;
    stamped.mask.label = Label::FIELD;
    stamped.mask.mask = mask;
    return stamped;
}

/** Camera range implied by a recovered pose: the field centre's distance from the camera. */
double range_of(const Eigen::Matrix4d &tf_camera_from_fieldcenter) {
    return tf_camera_from_fieldcenter.block<3, 1>(0, 3).norm();
}
}  // namespace

class HomographyFieldFilterTest : public ::testing::Test {};

TEST_F(HomographyFieldFilterTest, RecoversAnOverheadPoseWithinTenMillimetres) {
    const Eigen::Matrix4d truth = testing_support::camera_pose(3.0, 5.0, 12.0);
    const cv::Mat mask = testing_support::render_field_mask(
        truth, testing_support::test_intrinsics(), kImageSize, kFieldSize, kFieldSize);

    HomographyFieldFilter filter(make_config());
    const auto description = filter.compute_field(make_camera_data(), make_mask(mask));

    ASSERT_EQ(description->header.frame_id, FrameId::CAMERA_WORLD);
    const Eigen::Vector3d recovered = description->tf_camera_from_fieldcenter.tf.block<3, 1>(0, 3);
    const Eigen::Vector3d expected = truth.block<3, 1>(0, 3);
    EXPECT_LT((recovered - expected).norm(), 0.010);
    EXPECT_NEAR(description->size.size.x, kFieldSize, 1e-9);
    EXPECT_NEAR(description->size.size.y, kFieldSize, 1e-9);
}

TEST_F(HomographyFieldFilterTest, RecoversATripodPoseWithinTenMillimetres) {
    const Eigen::Matrix4d truth = testing_support::camera_pose(5.0, 45.0, -20.0);
    const cv::Mat mask = testing_support::render_field_mask(
        truth, testing_support::test_intrinsics(), kImageSize, kFieldSize, kFieldSize);

    HomographyFieldFilter filter(make_config());
    const auto description = filter.compute_field(make_camera_data(), make_mask(mask));

    ASSERT_EQ(description->header.frame_id, FrameId::CAMERA_WORLD);
    const Eigen::Vector3d recovered = description->tf_camera_from_fieldcenter.tf.block<3, 1>(0, 3);
    const Eigen::Vector3d expected = truth.block<3, 1>(0, 3);
    EXPECT_LT((recovered - expected).norm(), 0.010);
}

TEST_F(HomographyFieldFilterTest, RecoversTheFullRectangleThroughRoundedCorners) {
    // The regression test for the corner bug. approxPolyDP returns vertices that lie on the
    // outline, so where the mat's corners are rounded it chords across them: the quad came out
    // 19% small in area, which is 11% linearly and put the camera 11% too far away.
    const Eigen::Matrix4d truth = testing_support::camera_pose(3.5, 20.0, 8.0);
    const cv::Mat rounded = testing_support::render_field_mask(
        truth, testing_support::test_intrinsics(), kImageSize, kFieldSize, kFieldSize, 0.20);

    HomographyFieldFilter filter(make_config());
    const auto description = filter.compute_field(make_camera_data(), make_mask(rounded));
    ASSERT_EQ(description->header.frame_id, FrameId::CAMERA_WORLD);

    const double recovered = range_of(description->tf_camera_from_fieldcenter.tf);
    const double expected = range_of(truth);
    EXPECT_LT(std::abs(recovered - expected) / expected, 0.03)
        << "recovered " << recovered << " m against " << expected << " m";
}

TEST_F(HomographyFieldFilterTest, RejectsAnOutlineThatSpillsOutsideTheQuad) {
    // The guard the coverage limit exists for: an outline whose fitted quad chords through the
    // mask. That was the pre-refinement failure, where approxPolyDP inscribed the quad inside the
    // mat's rounded corners and the mask came out 24% larger than its own fit. Driving the limit
    // below a good fit's coverage is the way to exercise the wiring, because with the edge
    // refinement in place the fitted quad circumscribes the mask and the ratio no longer rises
    // above 1 on its own.
    const Eigen::Matrix4d truth = testing_support::camera_pose(3.0, 5.0, 12.0);
    const cv::Mat mask = testing_support::render_field_mask(
        truth, testing_support::test_intrinsics(), kImageSize, kFieldSize, kFieldSize);

    HomographyFieldFilterConfiguration config = make_config();
    config.max_quad_coverage = 0.5;
    HomographyFieldFilter filter(config);
    const auto description = filter.compute_field(make_camera_data(), make_mask(mask));
    EXPECT_EQ(description->header.frame_id, FrameId::EMPTY);
}

TEST_F(HomographyFieldFilterTest, ClippedAndWellFramedOutlinesReportComparableCoverage) {
    // Coverage is measured against the quad clipped to the image. Without that, a field running
    // off the frame has corners off the sensor, its quad is larger than anything the mask could
    // fill, and the ratio reads as a bad outline on exactly the case this filter exists for.
    const cv::Mat framed = testing_support::render_field_mask(
        testing_support::camera_pose(3.0, 5.0, 12.0), testing_support::test_intrinsics(),
        kImageSize, kFieldSize, kFieldSize);
    const cv::Mat clipped = testing_support::render_field_mask(
        testing_support::camera_pose(1.6, 55.0, 0.0), testing_support::test_intrinsics(),
        kImageSize, kFieldSize, kFieldSize);

    const FieldOutline framed_outline =
        extract_field_outline(find_largest_contour_mask(framed), FieldOutlineParams{});
    const FieldOutline clipped_outline =
        extract_field_outline(find_largest_contour_mask(clipped), FieldOutlineParams{});
    ASSERT_TRUE(framed_outline.ok) << framed_outline.failure;
    ASSERT_TRUE(clipped_outline.ok) << clipped_outline.failure;
    EXPECT_NEAR(clipped_outline.mask_over_quad_area, framed_outline.mask_over_quad_area, 0.10);
}

TEST_F(HomographyFieldFilterTest, RejectsAnEmptyMask) {
    HomographyFieldFilter filter(make_config());
    const cv::Mat empty = cv::Mat::zeros(kImageSize, CV_8UC1);
    const auto description = filter.compute_field(make_camera_data(), make_mask(empty));
    EXPECT_EQ(description->header.frame_id, FrameId::EMPTY);
}

TEST_F(HomographyFieldFilterTest, SolvesAClippedFieldFromThreeEdges) {
    // The expected case at a cage mount: at 87 degrees of rectilinear horizontal field the near
    // mat corners fall outside the frame, so there is no four-corner fit to make. Three edges
    // still determine the pose exactly.
    const Eigen::Matrix4d truth = testing_support::camera_pose(1.6, 55.0, 0.0);
    const cv::Mat mask = testing_support::render_field_mask(
        truth, testing_support::test_intrinsics(), kImageSize, kFieldSize, kFieldSize);

    FieldOutlineParams params;
    const FieldOutline outline = extract_field_outline(find_largest_contour_mask(mask), params);
    ASSERT_TRUE(outline.ok) << outline.failure;
    ASSERT_EQ(outline.supported_sides, 3)
        << "the synthetic pose is meant to run one field edge off the frame";

    HomographyFieldFilter filter(make_config());
    const auto description = filter.compute_field(make_camera_data(), make_mask(mask));
    ASSERT_EQ(description->header.frame_id, FrameId::CAMERA_WORLD);

    const Eigen::Vector3d recovered = description->tf_camera_from_fieldcenter.tf.block<3, 1>(0, 3);
    const Eigen::Vector3d expected = truth.block<3, 1>(0, 3);
    EXPECT_LT((recovered - expected).norm(), 0.030);
}

TEST_F(HomographyFieldFilterTest, ThreeEdgesAgreeWithFourCornersOnTheSameOutline) {
    // Same outline, both solvers. Dropping a side that is actually visible must not move the
    // answer, or the clipped path is measuring something different from the well-framed one.
    const Eigen::Matrix4d truth = testing_support::camera_pose(3.5, 30.0, 15.0);
    const cv::Mat mask = testing_support::render_field_mask(
        truth, testing_support::test_intrinsics(), kImageSize, kFieldSize, kFieldSize);
    const FieldOutline outline =
        extract_field_outline(find_largest_contour_mask(mask), FieldOutlineParams{});
    ASSERT_TRUE(outline.ok) << outline.failure;
    ASSERT_EQ(outline.supported_sides, 4);

    const FieldPoseResult four = pose_from_corners(outline.corners, kFieldSize, kFieldSize,
                                                   testing_support::test_intrinsics());
    ASSERT_TRUE(four.ok) << four.failure;

    for (size_t dropped = 0; dropped < 4; ++dropped) {
        std::array<bool, 4> supported = outline.side_supported;
        supported[dropped] = false;
        const FieldPoseResult three =
            pose_from_three_lines(outline.lines, supported, outline.corners, kFieldSize, kFieldSize,
                                  testing_support::test_intrinsics());
        ASSERT_TRUE(three.ok) << "dropping side " << dropped << ": " << three.failure;
        const Eigen::Vector3d difference = three.tf_camera_from_fieldcenter.block<3, 1>(0, 3) -
                                           four.tf_camera_from_fieldcenter.block<3, 1>(0, 3);
        EXPECT_LT(difference.norm(), 0.010) << "dropping side " << dropped;
    }
}
}  // namespace auto_battlebot
