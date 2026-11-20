#pragma once

#include <opencv2/core.hpp>

#include "mask_model/mask_model_interface.hpp"

namespace auto_battlebot {
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
