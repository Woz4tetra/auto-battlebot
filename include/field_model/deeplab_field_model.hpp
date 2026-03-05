#pragma once

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "enums/deeplab_model_type.hpp"
#include "field_model/config.hpp"
#include "field_model/field_model_interface.hpp"
#include "tensorrt_inference/trt_engine.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
class DeepLabFieldModel : public FieldModelInterface {
   public:
    DeepLabFieldModel(DeepLabFieldModelConfiguration &config);

    bool initialize() override;
    FieldMaskStamped update(RgbImage image) override;

   protected:
    std::string model_path_;
    DeepLabModelType model_type_;
    int image_size_ = 0;
    int border_padding_ = 0;
    TrtEngine engine_;
    bool initialized_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    bool load_model_config();

    // Helper methods: preprocess writes NCHW float into buffer; postprocess takes host float output
    // and dimensions.
    void preprocess_image(const cv::Mat &image, std::vector<float> &buffer);
    cv::Mat postprocess_output(const float *output, int original_height, int original_width);
};

}  // namespace auto_battlebot
