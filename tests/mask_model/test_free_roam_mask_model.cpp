#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "mask_model/free_roam_mask_model.hpp"

namespace auto_battlebot {
namespace {
constexpr int kWidth = 100;
constexpr int kHeight = 80;
const cv::Scalar kFieldColorBgr(120, 125, 130);
const cv::Scalar kBlobColorBgr(30, 30, 220);
const cv::Point kCenterPx(kWidth / 2, kHeight / 2);
const cv::Point kFarCornerPx(kWidth - 1, kHeight - 1);

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

    RgbImage image = make_image(kFieldColorBgr);
    MaskStamped result = model.update(image);

    ASSERT_EQ(result.mask.mask.rows, kHeight);
    ASSERT_EQ(result.mask.mask.cols, kWidth);
    EXPECT_EQ(result.mask.label, Label::FIELD);
    EXPECT_DOUBLE_EQ(result.header.stamp, image.header.stamp);

    EXPECT_EQ(result.mask.mask.at<uchar>(kCenterPx), 255);
    EXPECT_EQ(result.mask.mask.at<uchar>(0, 0), 0);
    EXPECT_EQ(result.mask.mask.at<uchar>(kFarCornerPx), 0);
    EXPECT_EQ(cv::countNonZero(result.mask.mask), (kWidth / 2) * (kHeight / 2));
}

TEST_F(FreeRoamMaskModelTest, RoiFullFrameFillsEverything) {
    config.color_filter = false;
    config.roi_fraction = 1.0;
    FreeRoamMaskModel model(config);

    MaskStamped result = model.update(make_image(kFieldColorBgr));

    EXPECT_EQ(cv::countNonZero(result.mask.mask), kWidth * kHeight);
}

TEST_F(FreeRoamMaskModelTest, EmptyImageReturnsEmptyMask) {
    FreeRoamMaskModel model(config);

    RgbImage image;
    image.header.stamp = 1.0;
    MaskStamped result = model.update(image);

    EXPECT_TRUE(result.mask.mask.empty());
}

TEST_F(FreeRoamMaskModelTest, DefaultsAreFullFrameRoiWithColorFilterOn) {
    EXPECT_DOUBLE_EQ(config.roi_fraction, 1.0);
    EXPECT_TRUE(config.color_filter);
}

TEST_F(FreeRoamMaskModelTest, ColorMatchExcludesDistinctBlob) {
    FreeRoamMaskModel model(config);
    const cv::Rect kOffCenterBlob(5, 5, 20, 20);
    const cv::Point kBlobCenterPx(15, 15);
    const cv::Point kFarFieldPx(kWidth - 5, kHeight - 5);

    RgbImage image = make_image(kFieldColorBgr);
    cv::rectangle(image.image, kOffCenterBlob, kBlobColorBgr, cv::FILLED);
    MaskStamped result = model.update(image);

    EXPECT_EQ(result.mask.mask.at<uchar>(kBlobCenterPx), 0);
    EXPECT_EQ(result.mask.mask.at<uchar>(kCenterPx), 255);
    EXPECT_EQ(result.mask.mask.at<uchar>(kFarFieldPx), 255);
}

TEST_F(FreeRoamMaskModelTest, LightingRampStaysInMask) {
    FreeRoamMaskModel model(config);

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

    RgbImage image = make_image(kFieldColorBgr);
    const cv::Rect kSeedPatch(kWidth / 2 - 10, kHeight / 2 - 8, 20, 16);
    constexpr double kBlobCoverage = 0.4;
    cv::Rect blob(kSeedPatch.x, kSeedPatch.y, static_cast<int>(kSeedPatch.width * kBlobCoverage),
                  kSeedPatch.height);
    cv::rectangle(image.image, blob, kBlobColorBgr, cv::FILLED);
    MaskStamped result = model.update(image);

    const cv::Point kFieldProbePx(kWidth - 5, 5);
    EXPECT_EQ(result.mask.mask.at<uchar>(kFieldProbePx), 255);
    EXPECT_EQ(result.mask.mask.at<uchar>(blob.y + blob.height / 2, blob.x + 1), 0);
}

TEST_F(FreeRoamMaskModelTest, RoiAndColorMustBothPass) {
    config.roi_fraction = 0.5;
    FreeRoamMaskModel model(config);

    RgbImage image = make_image(kFieldColorBgr);
    MaskStamped result = model.update(image);

    const cv::Point kOutsideRoiProbePx(2, 2);
    EXPECT_EQ(result.mask.mask.at<uchar>(kOutsideRoiProbePx), 0);
    EXPECT_EQ(result.mask.mask.at<uchar>(kCenterPx), 255);
}

}  // namespace auto_battlebot
