#pragma once

#include <torch/script.h>
#include <torch/cuda.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "field_model/field_model_interface.hpp"
#include "enums/deeplab_model_type.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "time_utils.hpp"

namespace auto_battlebot
{
    class DeepLabFieldModel : public FieldModelInterface
    {
    public:
        DeepLabFieldModel(const std::string &model_path, DeepLabModelType model_type, int image_size = 512, int border_crop_padding = 20);

        bool initialize() override;
        FieldMaskStamped update(RgbImage image) override;

    protected:
        std::string model_path_;
        DeepLabModelType model_type_;
        int image_size_;
        int border_crop_padding_;
        torch::jit::script::Module model_;
        torch::Device device_;
        bool initialized_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        // Helper methods
        torch::Tensor preprocess_image(const cv::Mat &image);
        cv::Mat postprocess_output(const torch::Tensor &output, int original_height, int original_width);
    };

} // namespace auto_battlebot
