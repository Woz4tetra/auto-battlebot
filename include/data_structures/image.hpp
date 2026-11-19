#pragma once

#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "header.hpp"

namespace auto_battlebot {
inline cv::Mat require_rgb_layout(cv::Mat image) {
    if (!image.empty() && image.type() != CV_8UC3) {
        throw std::invalid_argument("RgbImage requires a CV_8UC3 image");
    }
    return image;
}

inline cv::Mat require_depth_layout(cv::Mat image) {
    if (!image.empty() && image.type() != CV_32FC1) {
        throw std::invalid_argument("DepthImage requires a CV_32FC1 image");
    }
    return image;
}

struct RgbImage {
    Header header;
    cv::Mat image;
};

struct DepthImage {
    Header header;
    cv::Mat image;
};

}  // namespace auto_battlebot
