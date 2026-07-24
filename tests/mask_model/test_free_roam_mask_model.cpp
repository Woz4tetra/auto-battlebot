#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "mask_model/free_roam_mask_model.hpp"

namespace auto_battlebot {
namespace {
constexpr int kWidth = 100;
constexpr int kHeight = 80;
const cv::Scalar kFieldColor(120, 125, 130);  // BGR, neutral gray-ish field material
const cv::Scalar kBlobColor(30, 30, 220);     // BGR, distinctly red

RgbImage make_image(const cv::Scalar &fill) {
    RgbImage image;
    image.header.stamp = 123.456;
    image.header.frame_id = FrameId::CAMERA;
    image.image = cv::Mat(kHeight, kWidth, CV_8UC3, fill);
    return image;
}
}  // namespace

class FreeRoamMaskModelTest : public ::testing::Test {
   protected:
    FreeRoamMaskModelConfiguration config;
};

TEST_F(FreeRoamMaskModelTest, RoiOnlyMasksCenteredRectangle) {
    config.color_filter = false;
    config.roi_fraction = 0.5;
    FreeRoamMaskModel model(config);

    RgbImage image = make_image(kFieldColor);
    MaskStamped result = model.update(image);

    ASSERT_EQ(result.mask.mask.rows, kHeight);
    ASSERT_EQ(result.mask.mask.cols, kWidth);
    EXPECT_EQ(result.mask.label, Label::FIELD);
    EXPECT_DOUBLE_EQ(result.header.stamp, image.header.stamp);

    // Center is inside the 50% ROI, corners are outside.
    EXPECT_EQ(result.mask.mask.at<uchar>(kHeight / 2, kWidth / 2), 255);
    EXPECT_EQ(result.mask.mask.at<uchar>(0, 0), 0);
    EXPECT_EQ(result.mask.mask.at<uchar>(kHeight - 1, kWidth - 1), 0);
    EXPECT_EQ(cv::countNonZero(result.mask.mask), (kWidth / 2) * (kHeight / 2));
}

TEST_F(FreeRoamMaskModelTest, RoiFullFrameFillsEverything) {
    config.color_filter = false;
    config.roi_fraction = 1.0;
    FreeRoamMaskModel model(config);

    MaskStamped result = model.update(make_image(kFieldColor));

    EXPECT_EQ(cv::countNonZero(result.mask.mask), kWidth * kHeight);
}

TEST_F(FreeRoamMaskModelTest, EmptyImageReturnsEmptyMask) {
    FreeRoamMaskModel model(config);

    RgbImage image;
    image.header.stamp = 1.0;
    MaskStamped result = model.update(image);

    EXPECT_TRUE(result.mask.mask.empty());
}

TEST_F(FreeRoamMaskModelTest, ColorMatchExcludesDistinctBlob) {
    FreeRoamMaskModel model(config);  // defaults: full-frame ROI, color filter on

    RgbImage image = make_image(kFieldColor);
    // Off-center red blob, well away from the seed patch.
    cv::rectangle(image.image, cv::Rect(5, 5, 20, 20), kBlobColor, cv::FILLED);
    MaskStamped result = model.update(image);

    EXPECT_EQ(result.mask.mask.at<uchar>(15, 15), 0);                     // blob center excluded
    EXPECT_EQ(result.mask.mask.at<uchar>(kHeight / 2, kWidth / 2), 255);  // field included
    EXPECT_EQ(result.mask.mask.at<uchar>(kHeight - 5, kWidth - 5), 255);  // far corner included
}

TEST_F(FreeRoamMaskModelTest, LightingRampStaysInMask) {
    FreeRoamMaskModel model(config);

    // Horizontal brightness ramp on gray material: 90 on the left to 160 on the right.
    // This moves mostly L in Lab, which the loose tolerance_l must absorb.
    RgbImage image = make_image(cv::Scalar(0, 0, 0));
    for (int col = 0; col < kWidth; ++col) {
        int value = 90 + (160 - 90) * col / (kWidth - 1);
        image.image.col(col).setTo(cv::Scalar(value, value, value));
    }
    MaskStamped result = model.update(image);

    EXPECT_EQ(cv::countNonZero(result.mask.mask), kWidth * kHeight);
}

TEST_F(FreeRoamMaskModelTest, MedianSeedIgnoresBlobOverlappingPatch) {
    config.seed_patch_fraction = 0.2;
    FreeRoamMaskModel model(config);

    RgbImage image = make_image(kFieldColor);
    // Blob covering roughly the left 40% of the centered 20x16 seed patch.
    cv::Rect patch(kWidth / 2 - 10, kHeight / 2 - 8, 20, 16);
    cv::Rect blob(patch.x, patch.y, patch.width * 2 / 5, patch.height);
    cv::rectangle(image.image, blob, kBlobColor, cv::FILLED);
    MaskStamped result = model.update(image);

    // Median seed is still field color: field pixels match, the blob does not.
    EXPECT_EQ(result.mask.mask.at<uchar>(5, kWidth - 5), 255);
    EXPECT_EQ(result.mask.mask.at<uchar>(blob.y + blob.height / 2, blob.x + 1), 0);
}

TEST_F(FreeRoamMaskModelTest, RoiAndColorMustBothPass) {
    config.roi_fraction = 0.5;
    FreeRoamMaskModel model(config);

    RgbImage image = make_image(kFieldColor);
    MaskStamped result = model.update(image);

    // Field-colored pixel outside the ROI is excluded; inside is included.
    EXPECT_EQ(result.mask.mask.at<uchar>(2, 2), 0);
    EXPECT_EQ(result.mask.mask.at<uchar>(kHeight / 2, kWidth / 2), 255);
}

}  // namespace auto_battlebot
