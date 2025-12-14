#pragma once

#include <opencv2/opencv.hpp>

namespace auto_battlebot
{
    struct RgbImage
    {
        cv::Mat image; // WxHx3 uint8 array of pixels
    };

    struct DepthImage
    {
        cv::Mat image; // WxHx1 float array of depths
    };

} // namespace auto_battlebot
