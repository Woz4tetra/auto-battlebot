#include <gtest/gtest.h>

#include "data_structures/image.hpp"

namespace auto_battlebot {

TEST(ImageLayoutTest, RgbRejectsNonUint8ThreeChannel) {
    EXPECT_NO_THROW(require_rgb_layout(cv::Mat(4, 4, CV_8UC3)));
    EXPECT_NO_THROW(require_rgb_layout(cv::Mat()));
    EXPECT_THROW(require_rgb_layout(cv::Mat(4, 4, CV_8UC4)), std::invalid_argument);
}

TEST(ImageLayoutTest, DepthRejectsNonFloatSingleChannel) {
    EXPECT_NO_THROW(require_depth_layout(cv::Mat(4, 4, CV_32FC1)));
    EXPECT_NO_THROW(require_depth_layout(cv::Mat()));
    EXPECT_THROW(require_depth_layout(cv::Mat(4, 4, CV_8UC1)), std::invalid_argument);
}

}  // namespace auto_battlebot
