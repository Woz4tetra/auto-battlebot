#include <gtest/gtest.h>

#include "foxglove_adapters/camera_info.hpp"

namespace auto_battlebot {
class CameraCalibrationAdapterTest : public ::testing::Test {
   protected:
    void SetUp() override {
        test_camera_info.header.stamp = 123.456;
        test_camera_info.header.frame_id = FrameId::CAMERA;
        test_camera_info.width = 640;
        test_camera_info.height = 480;
        test_camera_info.intrinsics = cv::Mat::eye(3, 3, CV_64F);
        test_camera_info.intrinsics.at<double>(0, 0) = 500.0;
        test_camera_info.intrinsics.at<double>(1, 1) = 500.0;
        test_camera_info.intrinsics.at<double>(0, 2) = 320.0;
        test_camera_info.intrinsics.at<double>(1, 2) = 240.0;
        test_camera_info.distortion = cv::Mat(5, 1, CV_64F);
        test_camera_info.distortion.at<double>(0) = 0.1;
        test_camera_info.distortion.at<double>(1) = -0.2;
        test_camera_info.distortion.at<double>(2) = 0.01;
        test_camera_info.distortion.at<double>(3) = -0.01;
        test_camera_info.distortion.at<double>(4) = 0.05;
    }

    CameraInfo test_camera_info;
};

TEST_F(CameraCalibrationAdapterTest, BasicConversion) {
    auto calibration = foxglove_adapters::to_camera_calibration(test_camera_info);

    EXPECT_EQ(calibration.height, 480u);
    EXPECT_EQ(calibration.width, 640u);
    EXPECT_EQ(calibration.distortion_model, "plumb_bob");
    EXPECT_EQ(calibration.frame_id, "camera");
    ASSERT_TRUE(calibration.timestamp.has_value());
    EXPECT_EQ(calibration.timestamp->sec, 123u);

    EXPECT_DOUBLE_EQ(calibration.k[0], 500.0);
    EXPECT_DOUBLE_EQ(calibration.k[2], 320.0);
    EXPECT_DOUBLE_EQ(calibration.k[4], 500.0);
    EXPECT_DOUBLE_EQ(calibration.k[5], 240.0);
    EXPECT_DOUBLE_EQ(calibration.k[8], 1.0);

    ASSERT_EQ(calibration.d.size(), 5u);
    EXPECT_DOUBLE_EQ(calibration.d[0], 0.1);
    EXPECT_DOUBLE_EQ(calibration.d[4], 0.05);

    // R identity, P = [K | 0]
    for (int i = 0; i < 9; ++i) EXPECT_DOUBLE_EQ(calibration.r[i], (i % 4 == 0) ? 1.0 : 0.0);
    EXPECT_DOUBLE_EQ(calibration.p[0], 500.0);
    EXPECT_DOUBLE_EQ(calibration.p[2], 320.0);
    EXPECT_DOUBLE_EQ(calibration.p[3], 0.0);
    EXPECT_DOUBLE_EQ(calibration.p[6], 240.0);
    EXPECT_DOUBLE_EQ(calibration.p[10], 1.0);
    EXPECT_DOUBLE_EQ(calibration.p[11], 0.0);
}

TEST_F(CameraCalibrationAdapterTest, EmptyDistortion) {
    test_camera_info.distortion = cv::Mat();
    auto calibration = foxglove_adapters::to_camera_calibration(test_camera_info);
    EXPECT_EQ(calibration.d.size(), 0u);
    EXPECT_TRUE(calibration.distortion_model.empty());
}

TEST_F(CameraCalibrationAdapterTest, EncodesToProtobuf) {
    auto calibration = foxglove_adapters::to_camera_calibration(test_camera_info);
    std::vector<uint8_t> buffer(1024);
    size_t encoded = 0;
    ASSERT_EQ(calibration.encode(buffer.data(), buffer.size(), &encoded),
              foxglove::FoxgloveError::Ok);
    EXPECT_GT(encoded, 0u);
}

// The stamp is a uint64 above 2^53, so it must be a JSON string, never a number.
TEST(FrameMetaJsonTest, StampIsStringAndPathIsEscaped) {
    FrameIdentity identity;
    identity.image_stamp_ns = 1788011445339499712ull;
    identity.svo_frame_index = 42;
    identity.svo_path = "data/svo/a \"b\".svo2";
    EXPECT_EQ(foxglove_adapters::to_frame_meta_json(identity),
              "{\"image_stamp_ns\":\"1788011445339499712\",\"svo_frame_index\":42,"
              "\"svo_path\":\"data/svo/a \\\"b\\\".svo2\"}");
}

TEST(FrameMetaJsonTest, NoSvoRecording) {
    FrameIdentity identity;
    identity.image_stamp_ns = 7;
    EXPECT_EQ(foxglove_adapters::to_frame_meta_json(identity),
              "{\"image_stamp_ns\":\"7\",\"svo_frame_index\":-1,\"svo_path\":\"\"}");
}

}  // namespace auto_battlebot
