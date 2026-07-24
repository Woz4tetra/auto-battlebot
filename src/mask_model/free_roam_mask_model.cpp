#include "mask_model/free_roam_mask_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot {
namespace {
cv::Rect centered_rect(cv::Size image_size, double fraction) {
    int width = std::max(1, static_cast<int>(std::lround(image_size.width * fraction)));
    int height = std::max(1, static_cast<int>(std::lround(image_size.height * fraction)));
    return {(image_size.width - width) / 2, (image_size.height - height) / 2, width, height};
}
}  // namespace

FreeRoamMaskModel::FreeRoamMaskModel(const FreeRoamMaskModelConfiguration &config)
    : roi_fraction_(config.roi_fraction),
      color_filter_(config.color_filter),
      seed_patch_fraction_(config.seed_patch_fraction),
      tolerance_l_(config.tolerance_l),
      tolerance_ab_(config.tolerance_ab),
      debug_visualization_(config.debug_visualization),
      diagnostics_logger_(DiagnosticsLogger::get_logger("free_roam_mask_model")) {}

MaskStamped FreeRoamMaskModel::update(RgbImage image) {
    MaskStamped result;
    result.header = image.header;
    result.mask.label = Label::FIELD;
    if (image.image.empty()) {
        return result;
    }

    cv::Mat mask = make_roi_mask(image.image.size());

    cv::Rect seed_rect;
    cv::Mat color_mask;
    if (color_filter_) {
        cv::Mat lab_image;
        cv::cvtColor(image.image, lab_image, cv::COLOR_BGR2Lab);

        // Median, not mean: a robot or debris overlapping the seed patch must not skew the seed.
        seed_rect = centered_rect(image.image.size(), seed_patch_fraction_);
        cv::Vec3b seed = median_color(lab_image(seed_rect));

        cv::Scalar lower(seed[0] - tolerance_l_, seed[1] - tolerance_ab_, seed[2] - tolerance_ab_);
        cv::Scalar upper(seed[0] + tolerance_l_, seed[1] + tolerance_ab_, seed[2] + tolerance_ab_);
        cv::inRange(lab_image, lower, upper, color_mask);

        cv::bitwise_and(mask, color_mask, mask);
        // Close small speckle holes; the largest-contour pass downstream discards islands, so the
        // mask only needs one solid field region.
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        diagnostics_logger_->debug("color_seed", {{"seed_l", std::to_string(seed[0])},
                                                  {"seed_a", std::to_string(seed[1])},
                                                  {"seed_b", std::to_string(seed[2])}});
    }

    diagnostics_logger_->debug("mask", {{"nonzero_pixels", std::to_string(cv::countNonZero(mask))},
                                        {"total_pixels", std::to_string(mask.rows * mask.cols)}});

    if (debug_visualization_ && color_filter_) {
        visualize_debug_mosaic(image.image, seed_rect, color_mask, mask);
    }

    result.mask.mask = mask;
    return result;
}

cv::Mat FreeRoamMaskModel::make_roi_mask(cv::Size image_size) const {
    if (roi_fraction_ >= 1.0) {
        return {image_size, CV_8UC1, cv::Scalar(255)};
    }
    cv::Mat mask = cv::Mat::zeros(image_size, CV_8UC1);
    mask(centered_rect(image_size, roi_fraction_)) = 255;
    return mask;
}

cv::Vec3b FreeRoamMaskModel::median_color(const cv::Mat &patch) {
    std::array<std::array<int, 256>, 3> histograms{};
    for (int row = 0; row < patch.rows; ++row) {
        const auto *pixels = patch.ptr<cv::Vec3b>(row);
        for (int col = 0; col < patch.cols; ++col) {
            for (size_t channel = 0; channel < 3; ++channel) {
                ++histograms[channel][pixels[col][channel]];
            }
        }
    }

    int half_count = (patch.rows * patch.cols + 1) / 2;
    cv::Vec3b median;
    for (size_t channel = 0; channel < 3; ++channel) {
        int cumulative = 0;
        for (size_t value = 0; value < histograms[channel].size(); ++value) {
            cumulative += histograms[channel][value];
            if (cumulative >= half_count) {
                median[channel] = static_cast<uchar>(value);
                break;
            }
        }
    }
    return median;
}

void FreeRoamMaskModel::visualize_debug_mosaic(const cv::Mat &bgr_image, cv::Rect seed_rect,
                                               const cv::Mat &color_mask,
                                               const cv::Mat &final_mask) const {
    cv::Mat input_vis = bgr_image.clone();
    cv::rectangle(input_vis, seed_rect, cv::Scalar(0, 255, 0), 2);

    cv::Mat color_mask_vis, final_mask_vis;
    cv::cvtColor(color_mask, color_mask_vis, cv::COLOR_GRAY2BGR);
    cv::cvtColor(final_mask, final_mask_vis, cv::COLOR_GRAY2BGR);

    cv::putText(input_vis, "Input + Seed Patch", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
    cv::putText(color_mask_vis, "Color Mask", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);
    cv::putText(final_mask_vis, "Final Mask", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2);

    cv::Mat mosaic;
    cv::hconcat(std::vector<cv::Mat>{input_vis, color_mask_vis, final_mask_vis}, mosaic);
    cv::imshow("Free Roam Mask Debug", mosaic);
    cv::waitKey(1);
}

}  // namespace auto_battlebot
