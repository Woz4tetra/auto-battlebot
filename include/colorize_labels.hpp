#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_battlebot {
inline cv::Mat colorize_labels(const cv::Mat &label_mask) {
    if (label_mask.empty()) {
        return cv::Mat();
    }

    static const std::vector<cv::Vec3b> color_palette = {
        {0, 0, 0},       {0, 255, 0},   {0, 0, 255},    {255, 0, 0},     {0, 128, 0},
        {0, 0, 128},     {0, 128, 128}, {128, 0, 0},    {128, 0, 128},   {128, 128, 0},
        {128, 128, 128}, {0, 0, 64},    {0, 0, 192},    {0, 128, 64},    {0, 128, 192},
        {128, 0, 64},    {128, 0, 192}, {128, 128, 64}, {128, 128, 192}, {0, 64, 0},
        {0, 64, 128},    {0, 192, 0},   {0, 192, 128},  {128, 64, 0}};

    cv::Mat colorized(label_mask.size(), CV_8UC3);

    for (int y = 0; y < label_mask.rows; ++y) {
        for (int x = 0; x < label_mask.cols; ++x) {
            uint8_t label = label_mask.at<uint8_t>(y, x);

            size_t color_idx = label % color_palette.size();
            colorized.at<cv::Vec3b>(y, x) = color_palette[color_idx];
        }
    }

    return colorized;
}

}  // namespace auto_battlebot
