#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_battlebot
{
    /**
     * @brief Apply color map to a single-channel label mask
     *
     * Converts a grayscale mask where each pixel value represents a class label
     * into a color RGB image for visualization. Each label gets a distinct color.
     *
     * @param label_mask Single-channel uint8 mask with label values
     * @return cv::Mat 3-channel BGR image with colorized labels
     */
    inline cv::Mat colorize_labels(const cv::Mat &label_mask)
    {
        if (label_mask.empty())
        {
            return cv::Mat();
        }

        // Define a color palette
        // Each label gets a unique color. (BGR)
        static const std::vector<cv::Vec3b> color_palette = {
            {0, 0, 0},       // 0: Background (black)
            {0, 255, 0},     // Dark green
            {0, 0, 255},     // Dark red
            {255, 0, 0},     // Dark blue
            {0, 128, 0},     // Dark green
            {0, 0, 128},     // Dark red
            {0, 128, 128},   // Dark yellow
            {128, 0, 0},     // Dark blue
            {128, 0, 128},   // Dark magenta
            {128, 128, 0},   // Dark cyan
            {128, 128, 128}, // Gray
            {0, 0, 64},      // Maroon
            {0, 0, 192},     // Red
            {0, 128, 64},    // Olive
            {0, 128, 192},   // Orange
            {128, 0, 64},    // Purple
            {128, 0, 192},   // Pink
            {128, 128, 64},  // Teal
            {128, 128, 192}, // Light gray
            {0, 64, 0},      // Forest green
            {0, 64, 128},    // Brown
            {0, 192, 0},     // Lime
            {0, 192, 128},   // Yellow-green
            {128, 64, 0}     // Steel blue
        };

        cv::Mat colorized(label_mask.size(), CV_8UC3);

        for (int y = 0; y < label_mask.rows; ++y)
        {
            for (int x = 0; x < label_mask.cols; ++x)
            {
                uint8_t label = label_mask.at<uint8_t>(y, x);

                // Use modulo to handle labels beyond palette size
                size_t color_idx = label % color_palette.size();
                colorized.at<cv::Vec3b>(y, x) = color_palette[color_idx];
            }
        }

        return colorized;
    }

} // namespace auto_battlebot
