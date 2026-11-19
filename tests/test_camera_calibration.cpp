#include <gtest/gtest.h>

#include "rgbd_camera/camera_calibration.hpp"

namespace auto_battlebot {
namespace {

CameraCalibration make_calibration(int width, int height, double fx, double fy,
                                   const std::array<double, 5> &distortion) {
    CameraCalibration calibration;
    calibration.calibration_id = "test";
    calibration.width = width;
    calibration.height = height;
    calibration.fx = fx;
    calibration.fy = fy;
    calibration.cx = width / 2.0;
    calibration.cy = height / 2.0;
    calibration.distortion_k1_k2_p1_p2_k3 = distortion;
    return calibration;
}

TEST(RectifierTest, KeepsEverySourcePixelByDefault) {
    const CameraCalibration barrel = make_calibration(640, 480, 400.0, 400.0, {-0.3, 0.1, 0, 0, 0});
    Rectifier keep_all;
    keep_all.build(barrel, cv::Size(640, 480));
    Rectifier cropped;
    cropped.build(barrel, cv::Size(640, 480), 0.0);
    const double keep_all_fx = keep_all.camera_info().intrinsics.at<double>(0, 0);
    const double cropped_fx = cropped.camera_info().intrinsics.at<double>(0, 0);
    EXPECT_LT(keep_all_fx, cropped_fx) << "a cropping alpha cuts the mat edges the field fit needs";
}

TEST(RectifierTest, ScalesIntrinsicsToCaptureSize) {
    const CameraCalibration shot = make_calibration(1920, 1200, 1000.0, 900.0, {});
    Rectifier rectifier;
    rectifier.build(shot, cv::Size(1280, 720));
    const CameraInfo info = rectifier.camera_info();
    EXPECT_EQ(info.width, 1280);
    EXPECT_EQ(info.height, 720);
    EXPECT_NEAR(info.intrinsics.at<double>(0, 0), 1000.0 * 1280 / 1920, 1e-6);
    EXPECT_NEAR(info.intrinsics.at<double>(1, 1), 900.0 * 720 / 1200, 1e-6);
    EXPECT_NEAR(info.intrinsics.at<double>(0, 2), 640.0, 1e-6);
    EXPECT_NEAR(info.intrinsics.at<double>(1, 2), 360.0, 1e-6);
}

TEST(RectifierTest, PublishesZeroDistortionOfPlumbBobLength) {
    Rectifier rectifier;
    rectifier.build(make_calibration(640, 480, 400.0, 400.0, {-0.3, 0.1, 0, 0, 0}),
                    cv::Size(640, 480));
    const CameraInfo info = rectifier.camera_info();
    ASSERT_EQ(info.distortion.total(), 5u);
    EXPECT_EQ(cv::countNonZero(info.distortion), 0);
}

}  // namespace
}  // namespace auto_battlebot
