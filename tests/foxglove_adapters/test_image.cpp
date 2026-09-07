#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>

#include "foxglove_adapters/image.hpp"

namespace auto_battlebot {

TEST(CompressedImageAdapterTest, EncodesJpegWithHeader) {
    RgbImage rgb;
    rgb.header.stamp = 123.456;
    rgb.header.frame_id = FrameId::CAMERA;
    rgb.image = cv::Mat(32, 48, CV_8UC3, cv::Scalar(50, 100, 150));

    auto image = foxglove_adapters::to_compressed_image(rgb);

    EXPECT_EQ(image.format, "jpeg");
    EXPECT_EQ(image.frame_id, "camera");
    ASSERT_TRUE(image.timestamp.has_value());
    EXPECT_EQ(image.timestamp->sec, 123u);
    ASSERT_GT(image.data.size(), 2u);
    // JPEG SOI marker
    EXPECT_EQ(static_cast<uint8_t>(image.data[0]), 0xFF);
    EXPECT_EQ(static_cast<uint8_t>(image.data[1]), 0xD8);

    std::vector<uint8_t> bytes(image.data.size());
    for (size_t i = 0; i < bytes.size(); ++i) bytes[i] = static_cast<uint8_t>(image.data[i]);
    cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
    EXPECT_EQ(decoded.rows, 32);
    EXPECT_EQ(decoded.cols, 48);
}

TEST(CompressedImageAdapterTest, EmptyImageHasEmptyPayload) {
    RgbImage rgb;
    rgb.header.frame_id = FrameId::CAMERA_WORLD;
    auto image = foxglove_adapters::to_compressed_image(rgb);
    EXPECT_TRUE(image.data.empty());
    EXPECT_EQ(image.frame_id, "camera_world");
}

}  // namespace auto_battlebot
