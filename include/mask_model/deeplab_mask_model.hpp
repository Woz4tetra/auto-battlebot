#pragma once

#include <array>
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

inline constexpr std::array<float, 3> kDeepLabTrainingMeanRgb = {0.4611f, 0.4359f, 0.3905f};
inline constexpr std::array<float, 3> kDeepLabTrainingStdRgb = {0.2193f, 0.2150f, 0.2109f};

void deeplab_input_nchw(const cv::Mat &bgr_image, int model_height, int model_width,
                        int border_padding, std::vector<float> &buffer);

class DeepLabMaskModel : public MaskModelInterface {
   public:
    DeepLabMaskModel(const DeepLabMaskModelConfiguration &config,
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
