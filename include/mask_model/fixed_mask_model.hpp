#pragma once

#include <opencv2/core.hpp>

#include "mask_model/mask_model_interface.hpp"

namespace auto_battlebot {
/**
 * @brief Mask model that returns a constant non-empty FIELD mask.
 *
 * Used by the headless kinematic sim: the Runner's field-init gate requires a non-empty mask, but
 * the actual field size comes from FixedFieldFilter, not from perception. This satisfies the gate
 * without running a real segmentation model.
 */
class FixedMaskModel : public MaskModelInterface {
   public:
    bool initialize() override { return true; }

    MaskStamped update(RgbImage image) override {
        MaskStamped out;
        out.header = image.header;
        out.mask.label = Label::FIELD;
        out.mask.mask = cv::Mat::ones(2, 2, CV_8UC1);
        return out;
    }
};

}  // namespace auto_battlebot
