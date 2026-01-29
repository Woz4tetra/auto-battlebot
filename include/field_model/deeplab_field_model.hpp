#pragma once

#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "field_model/field_model_interface.hpp"
#include "enums/deeplab_model_type.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "field_model/config.hpp"
#include "tensorrt_inference/trt_engine.hpp"
#include "time_utils.hpp"

namespace auto_battlebot
{
    class DeepLabFieldModel : public FieldModelInterface
    {
    public:
        DeepLabFieldModel(DeepLabFieldModelConfiguration &config);

        bool initialize() override;
        FieldMaskStamped update(RgbImage image) override;

    protected:
        std::string model_path_;
        DeepLabModelType model_type_;
        int image_size_;
        int border_padding_;
        TrtEngine engine_;
        bool initialized_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        // Helper methods: preprocess writes NCHW float into buffer; postprocess takes host float output and dimensions.
        void preprocess_image(const cv::Mat &image, std::vector<float> &buffer);
        cv::Mat postprocess_output(const float *output, int original_height, int original_width);
    };

} // namespace auto_battlebot
