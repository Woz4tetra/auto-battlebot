#pragma once

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "engine_selector/engine_selector.hpp"
#include "enums/deeplab_model_type.hpp"
#include "mask_model/config.hpp"
#include "mask_model/mask_model_interface.hpp"
#include "tensorrt_inference/trt_engine.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
class DeepLabMaskModel : public MaskModelInterface {
   public:
    DeepLabMaskModel(DeepLabMaskModelConfiguration &config,
                     std::shared_ptr<EngineSelector> engine_selector);

    bool initialize() override;
    MaskStamped update(RgbImage image) override;

   protected:
    std::string model_path_;
    DeepLabModelType model_type_;
    Label output_label_;
    int image_size_ = 0;
    int border_padding_ = 0;
    TrtEngine engine_;
    std::shared_ptr<EngineSelector> engine_selector_;
    bool initialized_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    bool load_model_config();

    void preprocess_image_to_nchw_float(const cv::Mat &image, std::vector<float> &buffer);
    cv::Mat postprocess_host_output(const float *output, int original_height, int original_width);
};

}  // namespace auto_battlebot
