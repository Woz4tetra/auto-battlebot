#pragma once

#include "header.hpp"
#include <opencv2/opencv.hpp>

namespace auto_battlebot
{
    struct RgbImage
    {
        Header header;
        cv::Mat image; // WxHx3 uint8 array of pixels
    };

    struct DepthImage
    {
        Header header;
        cv::Mat image; // WxHx1 float array of depths
    };

} // namespace auto_battlebot
