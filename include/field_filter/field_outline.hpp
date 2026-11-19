#pragma once

#include <array>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

namespace auto_battlebot {
cv::Mat find_largest_contour_mask(const cv::Mat &mask);

struct FieldOutlineParams {
    int border_margin_px = 2;
    int refine_iterations = 4;
    double corner_skip_fraction_per_side_end = 0.20;
    int min_side_points = 20;
};

struct FieldOutline {
    bool ok = false;
    std::string failure;
    std::array<cv::Point2d, 4> corners{};
    std::array<cv::Vec3d, 4> lines{};
    std::array<bool, 4> side_supported{};
    std::array<std::optional<double>, 4> side_rms_px{};
    int supported_sides = 0;
    double mask_over_quad_area = 0.0;
};

FieldOutline extract_field_outline(const cv::Mat &contour_mask, const FieldOutlineParams &params);
}  // namespace auto_battlebot
