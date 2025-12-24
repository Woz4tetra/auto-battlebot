#pragma once

#include <torch/script.h>
#include <torch/cuda.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "enums/label.hpp"
#include <magic_enum.hpp>
#include <sstream>
#include <iomanip>
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
        float iou_threshold_;
        float letterbox_padding_;
        int image_size_;
        bool debug_visualization_;
        LabelToKeypointMapConfiguration label_map_;

        torch::jit::script::Module model_;
        torch::Device device_;
        bool initialized_;
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        // Helper methods
        float generate_scale(cv::Mat &image, const std::vector<int> &target_size);
        float letterbox(cv::Mat &input_image, cv::Mat &output_image, const std::vector<int> &target_size);
        torch::Tensor preprocess_image(const cv::Mat &image, cv::Size input_image_size);

        torch::Tensor nms(const torch::Tensor &bboxes, const torch::Tensor &scores, float iou_threshold);
        torch::Tensor non_max_suppression(torch::Tensor &prediction, int num_keypoints, float conf_thres = 0.25, float iou_thres = 0.45, int max_det = 300);
        torch::Tensor xywh2xyxy(const torch::Tensor &x);
        Keypoint scale_keypoint(Keypoint output_keypoint, cv::Size original_image_size, cv::Size input_image_size);
        torch::Tensor scale_boxes(torch::Tensor &boxes, cv::Size original_image_size, cv::Size input_image_size);
        KeypointsStamped postprocess_output(const torch::Tensor &output, const Header &header, cv::Size original_image_size, cv::Size input_image_size, const cv::Mat &original_image);
        void visualize_output(const cv::Mat &original_image, const KeypointsStamped &keypoints, const torch::Tensor &detections);

        // Visualization helpers
        static cv::Scalar get_color_for_label(Label label);
        static std::string get_short_name(const std::string &enum_name);
    };

} // namespace auto_battlebot
