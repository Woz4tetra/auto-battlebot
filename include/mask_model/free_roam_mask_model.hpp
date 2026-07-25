#pragma once

#include <memory>
#include <opencv2/core.hpp>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "mask_model/config.hpp"
#include "mask_model/mask_model_interface.hpp"

namespace auto_battlebot {
/**
 * @brief "Free roam" mask model: selects field pixels with classical heuristics instead of a
 * segmentation model, so PointCloudFieldFilter can fit whatever flat surface the camera points at.
 *
 * Two composable selectors, ANDed together to form the mask:
 * - Center ROI: centered rectangle covering roi_fraction of the frame (1.0 = full frame).
 * - Seeded color match: the median Lab color of a small center patch is the reference, and the
 *   whole frame is thresholded per channel against it. Lighting gradients and shadows mostly move
 *   L, so tolerance_l is set very loose while tolerance_ab stays tighter. Downstream RANSAC
 *   rejects same-colored non-plane pixels, so the mask only needs to be roughly right.
 */
class FreeRoamMaskModel : public MaskModelInterface {
   public:
    explicit FreeRoamMaskModel(const FreeRoamMaskModelConfiguration &config);

    bool initialize() override { return true; }
    MaskStamped update(RgbImage image) override;

   private:
    cv::Mat make_roi_mask(cv::Size image_size) const;
    static cv::Vec3b median_color(const cv::Mat &patch);
    void visualize_debug_mosaic(const cv::Mat &bgr_image, cv::Rect seed_rect,
                                const cv::Mat &color_mask, const cv::Mat &final_mask) const;

    double roi_fraction_;
    bool color_filter_;
    double seed_patch_fraction_;
    double tolerance_l_;
    double tolerance_ab_;
    bool debug_visualization_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;
};

}  // namespace auto_battlebot
