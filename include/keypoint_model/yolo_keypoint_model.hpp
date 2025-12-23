#pragma once

#include <torch/script.h>
#include <torch/cuda.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "keypoint_model/keypoint_model_interface.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "keypoint_model/config.hpp"
#include "config/enum_map_config.hpp"
#include "enums/label.hpp"
#include "enums/keypoint_label.hpp"
#include "time_utils.hpp"

namespace auto_battlebot
{
    class YoloKeypointModel : public KeypointModelInterface
    {
    public:
        YoloKeypointModel(YoloKeypointModelConfiguration &config);

        bool initialize() override;
        KeypointsStamped update(RgbImage image) override;

    protected:
        std::string model_path_;
        float threshold_;
        int image_size_;
        LabelToKeypointMapConfiguration label_map_;
        torch::jit::script::Module model_;
        torch::Device device_;
        bool initialized_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        // Helper methods
        torch::Tensor preprocess_image(const cv::Mat &image);
        KeypointsStamped postprocess_output(const torch::Tensor &output, const Header &header);
    };

} // namespace auto_battlebot
