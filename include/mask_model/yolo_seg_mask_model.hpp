#pragma once

#include <memory>
#include <string>
#include <vector>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "enums/label.hpp"
#include "mask_model/config.hpp"
#include "mask_model/mask_model_interface.hpp"
#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
class YoloSegMaskModel : public MaskModelInterface {
   public:
    explicit YoloSegMaskModel(YoloSegMaskModelConfiguration &config);

    bool initialize() override;
    MaskStamped update(RgbImage image) override;

   private:
    struct Detection {
        float x1 = 0.0f;
        float y1 = 0.0f;
        float x2 = 0.0f;
        float y2 = 0.0f;
        float confidence = 0.0f;
        int class_id = 0;
        std::vector<float> mask_coeffs;
    };

    std::string model_path_;
    float confidence_threshold_;
    float iou_threshold_;
    float mask_threshold_;
    float letterbox_padding_;
    int image_size_;
    int max_detections_;
    bool debug_visualization_;
    Label output_label_;
    std::vector<Label> label_indices_;
    int target_class_index_ = -1;
    uint8_t output_mask_value_ = 1;

    TrtEngine engine_;
    bool initialized_ = false;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    void preprocess_image(const cv::Mat &image, cv::Size input_size, std::vector<float> &buffer);
    float letterbox(cv::Mat &input_image, cv::Mat &output_image,
                    const std::vector<int> &target_size);
    float generate_scale(cv::Mat &image, const std::vector<int> &target_size);

    std::vector<Detection> decode_detections(const std::vector<float> &det_output,
                                             const std::vector<int64_t> &det_shape,
                                             int proto_channels) const;
    std::vector<Detection> non_max_suppression(const std::vector<Detection> &detections) const;
    static float iou(const Detection &lhs, const Detection &rhs);

    cv::Mat decode_instance_mask(const Detection &det, const std::vector<float> &proto_output,
                                 const std::vector<int64_t> &proto_shape,
                                 cv::Size input_size) const;
    cv::Mat map_mask_to_original_image(const cv::Mat &input_mask, cv::Size original_size,
                                       cv::Size input_size) const;
};
}  // namespace auto_battlebot
