#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/core.hpp>

#include "viz/full_range_i420.hpp"

namespace auto_battlebot {
namespace {

cv::Vec3b jfif_decode(const cv::Mat &i420, int width, int height) {
    const double y = i420.at<unsigned char>(0, 0);
    const double cb = i420.ptr<unsigned char>()[static_cast<size_t>(width) * height];
    const double cr = i420.ptr<unsigned char>()[static_cast<size_t>(width) * height * 5 / 4];
    const auto clamp8 = [](double v) {
        return static_cast<unsigned char>(std::clamp(std::lround(v), 0L, 255L));
    };
    return {clamp8(y + 1.772 * (cb - 128.0)),
            clamp8(y - 0.344136 * (cb - 128.0) - 0.714136 * (cr - 128.0)),
            clamp8(y + 1.402 * (cr - 128.0))};
}

TEST(FullRangeI420Test, FlatColorsDecodeAsJfifFullRange) {
    const std::array<cv::Vec3b, 6> colors = {
        cv::Vec3b(0, 0, 0),   cv::Vec3b(255, 255, 255), cv::Vec3b(128, 128, 128),
        cv::Vec3b(255, 0, 0), cv::Vec3b(0, 255, 0),     cv::Vec3b(0, 0, 255),
    };
    for (const cv::Vec3b &color : colors) {
        const cv::Mat input(48, 64, CV_8UC3, cv::Scalar(color[0], color[1], color[2]));
        FullRangeI420Converter converter;
        const cv::Mat *i420 = converter.convert(input);
        ASSERT_NE(i420, nullptr);
        const cv::Vec3b decoded = jfif_decode(*i420, converter.width(), converter.height());
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(decoded[c], color[c], 2)
                << "color " << int(color[0]) << "," << int(color[1]) << "," << int(color[2])
                << " channel " << c;
        }
    }
}

TEST(FullRangeI420Test, WhiteUsesTheFullLumaRange) {
    const cv::Mat white(48, 64, CV_8UC3, cv::Scalar(255, 255, 255));
    FullRangeI420Converter converter;
    const cv::Mat *i420 = converter.convert(white);
    ASSERT_NE(i420, nullptr);
    EXPECT_EQ(i420->at<unsigned char>(0, 0), 255);
}

TEST(FullRangeI420Test, OddSizesCropToEven) {
    const cv::Mat input(47, 63, CV_8UC3, cv::Scalar(10, 20, 30));
    FullRangeI420Converter converter;
    const cv::Mat *i420 = converter.convert(input);
    ASSERT_NE(i420, nullptr);
    EXPECT_EQ(converter.width(), 62);
    EXPECT_EQ(converter.height(), 46);
    EXPECT_EQ(i420->rows, 46 * 3 / 2);
}

TEST(FullRangeI420Test, RejectsEmptyAndNonBgrInput) {
    FullRangeI420Converter converter;
    EXPECT_EQ(converter.convert(cv::Mat()), nullptr);
    EXPECT_EQ(converter.convert(cv::Mat(8, 8, CV_8UC1)), nullptr);
    EXPECT_EQ(converter.convert(cv::Mat(1, 1, CV_8UC3)), nullptr);
}

}  // namespace
}  // namespace auto_battlebot
