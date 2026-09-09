#include <gtest/gtest.h>

#include <opencv2/objdetect/aruco_detector.hpp>

#include "field_filter/config.hpp"
#include "field_filter/fiducial_field_filter.hpp"
#include "field_filter/synthetic_field.hpp"

namespace auto_battlebot {
namespace {
constexpr int kCols = 3;
constexpr int kRows = 5;
constexpr int kFirstId = 160;
constexpr double kMarkerSize = 0.065;
constexpr double kMarkerSeparation = 0.015;
constexpr double kFieldSize = 2.35;
const cv::Size kImageSize(1920, 1200);

FiducialFieldFilterConfiguration make_config() {
    FiducialFieldFilterConfiguration config;
    config.field_size_x = kFieldSize;
    config.field_size_y = kFieldSize;
    // One frame, so the test does not have to feed ten copies of a synthetic render.
    config.accumulate_frames = 1;
    return config;
}

/** Render the board as the camera would see it, by warping a generated board image onto the
 *  field plane through the pose under test. */
cv::Mat render_board(const Eigen::Matrix4d &tf_camera_from_fieldcenter,
                     const Eigen::Matrix4d &tf_fieldcenter_from_board) {
    const cv::aruco::Dictionary dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    const std::vector<int> ids = floor_board_ids(kCols, kRows, kFirstId);
    const cv::aruco::GridBoard board(cv::Size(kCols, kRows), static_cast<float>(kMarkerSize),
                                     static_cast<float>(kMarkerSeparation), dictionary,
                                     cv::Mat(ids, true));

    // A board image at a known metres-per-pixel, so its four corners have known board coordinates.
    constexpr double kPixelsPerMetre = 4000.0;
    const double board_width = kCols * kMarkerSize + (kCols - 1) * kMarkerSeparation;
    const double board_height = kRows * kMarkerSize + (kRows - 1) * kMarkerSeparation;
    const cv::Size board_pixels(static_cast<int>(board_width * kPixelsPerMetre),
                                static_cast<int>(board_height * kPixelsPerMetre));
    cv::Mat board_image;
    board.generateImage(board_pixels, board_image, 0, 1);
    cv::cvtColor(board_image, board_image, cv::COLOR_GRAY2BGR);

    const std::vector<cv::Point2f> source = {
        {0.0F, 0.0F},
        {static_cast<float>(board_pixels.width), 0.0F},
        {static_cast<float>(board_pixels.width), static_cast<float>(board_pixels.height)},
        {0.0F, static_cast<float>(board_pixels.height)}};
    const std::vector<Eigen::Vector3d> board_corners = {{0.0, 0.0, 0.0},
                                                        {board_width, 0.0, 0.0},
                                                        {board_width, board_height, 0.0},
                                                        {0.0, board_height, 0.0}};
    std::vector<Eigen::Vector3d> in_field;
    for (const auto &corner : board_corners) {
        in_field.push_back(tf_fieldcenter_from_board.block<3, 3>(0, 0) * corner +
                           tf_fieldcenter_from_board.block<3, 1>(0, 3));
    }
    const auto projected = testing_support::project(in_field, tf_camera_from_fieldcenter,
                                                    testing_support::test_intrinsics());
    std::vector<cv::Point2f> destination;
    for (const auto &point : projected) {
        destination.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
    }

    cv::Mat frame(kImageSize, CV_8UC3, cv::Scalar(90, 90, 90));
    cv::Mat warped;
    const cv::Mat homography = cv::getPerspectiveTransform(source, destination);
    cv::warpPerspective(board_image, warped, homography, kImageSize, cv::INTER_LINEAR,
                        cv::BORDER_TRANSPARENT);
    cv::Mat occupancy = cv::Mat::zeros(kImageSize, CV_8UC1);
    cv::fillPoly(occupancy,
                 std::vector<std::vector<cv::Point>>{
                     {projected[0], projected[1], projected[2], projected[3]}},
                 cv::Scalar(255));
    warped.copyTo(frame, occupancy);
    return frame;
}
}  // namespace

TEST(FiducialBoardIdsTest, ReversesEachRowToMatchThePrintedBoard) {
    // The manufactured board numbers markers right to left within a row; GridBoard fills them
    // left to right. Pairing them the wrong way round gives a reflection rather than a rotation,
    // and a reflection still decodes: 440 px of reprojection error against 16 px when correct.
    // Verified against a photo of the real board: with first_id 160 the top-left tag is 162 and
    // the top-right is 160.
    const std::vector<int> ids = floor_board_ids(3, 5, 160);
    ASSERT_EQ(ids.size(), 15U);
    EXPECT_EQ(ids[0], 162) << "top-left grid cell";
    EXPECT_EQ(ids[1], 161);
    EXPECT_EQ(ids[2], 160) << "top-right grid cell";
    EXPECT_EQ(ids[3], 165) << "second row, left cell";
    EXPECT_EQ(ids[14], 172) << "bottom-right grid cell";

    std::vector<int> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    for (size_t i = 0; i < sorted.size(); ++i) {
        EXPECT_EQ(sorted[i], 160 + static_cast<int>(i)) << "every id appears exactly once";
    }
}

TEST(FiducialFieldFilterTest, PlacesTheBoardOriginAtTheConfiguredCorner) {
    FiducialFieldFilterConfiguration config = make_config();
    config.corner = FieldCorner::NEG_X_NEG_Y;
    config.board_offset_x = 0.10;
    config.board_offset_y = 0.15;
    FiducialFieldFilter filter(config);

    const Eigen::Matrix4d tf = filter.tf_fieldcenter_from_board();
    EXPECT_NEAR(tf(0, 3), -kFieldSize / 2.0 + 0.10, 1e-9);
    EXPECT_NEAR(tf(1, 3), -kFieldSize / 2.0 + 0.15, 1e-9);
    EXPECT_NEAR(tf(2, 3), 0.0, 1e-9);

    config.corner = FieldCorner::POS_X_POS_Y;
    FiducialFieldFilter opposite(config);
    const Eigen::Matrix4d tf_opposite = opposite.tf_fieldcenter_from_board();
    EXPECT_NEAR(tf_opposite(0, 3), kFieldSize / 2.0 + 0.10, 1e-9);
    EXPECT_NEAR(tf_opposite(1, 3), kFieldSize / 2.0 + 0.15, 1e-9);
}

TEST(FiducialFieldFilterTest, RecoversTheFieldPoseFromARenderedBoard) {
    FiducialFieldFilterConfiguration config = make_config();
    config.corner = FieldCorner::NEG_X_NEG_Y;
    config.board_offset_x = 0.20;
    config.board_offset_y = 0.20;
    FiducialFieldFilter filter(config);

    // A near corner, which is where the board has to go: at the far corner a 65 mm marker is
    // 8 px after foreshortening and will not decode.
    const Eigen::Matrix4d truth = testing_support::camera_pose(2.4, 35.0, 0.0);
    const cv::Mat frame = render_board(truth, filter.tf_fieldcenter_from_board());

    CameraData camera_data;
    camera_data.camera_info.width = kImageSize.width;
    camera_data.camera_info.height = kImageSize.height;
    camera_data.camera_info.intrinsics = testing_support::test_intrinsics_cv();
    camera_data.rgb.image = frame;
    camera_data.rgb.header.stamp = 4.0;

    MaskStamped unused_mask;
    const auto description = filter.compute_field(camera_data, unused_mask);
    ASSERT_EQ(description->header.frame_id, FrameId::CAMERA_WORLD)
        << "the board should have been detected and the field locked";

    const Eigen::Vector3d recovered = description->tf_camera_from_fieldcenter.tf.block<3, 1>(0, 3);
    const Eigen::Vector3d expected = truth.block<3, 1>(0, 3);
    // 40 mm at 2.4 m is 1.7%, which is the synthetic render's own limit rather than the solver's.
    // The board is rasterized and then perspective-warped down to markers about 30 px on a side,
    // so their corners carry a fraction of a pixel of resampling bias that subpixel refinement
    // cannot undo. The residual is the number that says whether the fit is sound, and it comes
    // out under 1 px against the 3 px guard.
    EXPECT_LT((recovered - expected).norm(), 0.04)
        << "recovered " << recovered.transpose() << " against " << expected.transpose();
}

TEST(FiducialFieldFilterTest, RejectsAFrameWithNoBoard) {
    FiducialFieldFilter filter(make_config());
    CameraData camera_data;
    camera_data.camera_info.intrinsics = testing_support::test_intrinsics_cv();
    camera_data.rgb.image = cv::Mat(kImageSize, CV_8UC3, cv::Scalar(90, 90, 90));

    MaskStamped unused_mask;
    const auto description = filter.compute_field(camera_data, unused_mask);
    EXPECT_EQ(description->header.frame_id, FrameId::EMPTY);
}
}  // namespace auto_battlebot
