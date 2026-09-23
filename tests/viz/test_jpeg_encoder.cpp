#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "viz/jpeg_encoder.hpp"

namespace auto_battlebot {
namespace {

cv::Mat gradient(int width, int height) {
    cv::Mat image(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(static_cast<uint8_t>(x * 255 / width),
                                                  static_cast<uint8_t>(y * 255 / height), 128);
        }
    }
    return image;
}

// Whichever encoder make_jpeg_encoder() picks: NVJPG on the Jetson, OpenCV on the desktop.
TEST(JpegEncoderTest, RoundTripsAtInputSizeWithLowError) {
    auto encoder = make_jpeg_encoder();
    const cv::Mat input = gradient(640, 360);
    std::vector<std::byte> jpeg;
    ASSERT_TRUE(encoder->encode(input, 90, jpeg)) << encoder->name();
    ASSERT_FALSE(jpeg.empty());

    std::vector<uint8_t> bytes(reinterpret_cast<const uint8_t *>(jpeg.data()),
                               reinterpret_cast<const uint8_t *>(jpeg.data() + jpeg.size()));
    const cv::Mat decoded = cv::imdecode(bytes, cv::IMREAD_COLOR);
    ASSERT_EQ(decoded.size(), input.size());
    cv::Mat diff;
    cv::absdiff(decoded, input, diff);
    const cv::Scalar mean = cv::mean(diff);
    EXPECT_LT((mean[0] + mean[1] + mean[2]) / 3.0, 3.0) << encoder->name();
}

TEST(JpegEncoderTest, EmptyImageFails) {
    auto encoder = make_jpeg_encoder();
    std::vector<std::byte> jpeg;
    EXPECT_FALSE(encoder->encode(cv::Mat(), 90, jpeg));
    EXPECT_TRUE(jpeg.empty());
}

}  // namespace
}  // namespace auto_battlebot
