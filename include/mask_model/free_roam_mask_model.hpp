#pragma once

#include <memory>
#include <opencv2/core.hpp>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "mask_model/config.hpp"
#include "mask_model/mask_model_interface.hpp"

namespace auto_battlebot {
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
