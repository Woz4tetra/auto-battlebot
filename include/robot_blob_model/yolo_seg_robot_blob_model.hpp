#pragma once

#include <set>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "robot_blob_model/config.hpp"
#include "robot_blob_model/rectangle_keypoint_helpers.hpp"
#include "robot_blob_model/robot_blob_model_interface.hpp"
#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
class YoloSegRobotBlobModel : public RobotBlobModelInterface {
   public:
    explicit YoloSegRobotBlobModel(YoloSegRobotBlobModelConfiguration &config);

    bool initialize() override;
    KeypointsStamped update(RgbImage image) override;

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

    enum class Category { THEIR, NEUTRAL, FIELD, OTHER };

    std::string model_path_;
    float confidence_threshold_;
    float iou_threshold_;
    float mask_threshold_;
    float letterbox_padding_;
    int image_size_;
    int max_detections_;
    bool debug_visualization_;
    std::vector<Label> label_indices_;
    std::set<Label> their_robot_labels_;
    std::set<Label> neutral_robot_labels_;
    std::set<Label> field_labels_;

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
    bool select_output_tensors(const std::vector<TrtEngine::OutputTensorInfo> &output_infos,
                               size_t &det_idx, size_t &proto_idx, int &proto_channels) const;

    cv::Mat decode_instance_mask(const Detection &det, const std::vector<float> &proto_output,
                                 const std::vector<int64_t> &proto_shape,
                                 cv::Size input_size) const;
    cv::Mat map_mask_to_original_image(const cv::Mat &input_mask, cv::Size original_size,
                                       cv::Size input_size) const;
    cv::Mat decode_mapped_instance_mask(const Detection &det,
                                        const std::vector<std::vector<float>> &output_buffers,
                                        const std::vector<TrtEngine::OutputTensorInfo> &output_infos,
                                        size_t proto_idx, cv::Size original_size,
                                        cv::Size input_size) const;
    void map_detection_box_to_original(const Detection &det, cv::Size original_size, cv::Size input_size,
                                       int &x1, int &y1, int &x2, int &y2) const;
    cv::Scalar debug_color_for_detection(const Detection &det) const;
    void render_detection_debug(cv::Mat &debug_vis, const Detection &det, const cv::Mat &instance_mask,
                                cv::Size original_size, cv::Size input_size) const;
    static void render_keypoints_debug(cv::Mat &debug_vis, const KeypointsStamped &keypoints);
    Category classify_category(Label label) const;

    void append_detection_keypoints(const Detection &det, const cv::Mat &instance_mask,
                                    cv::Size original_size, cv::Size input_size,
                                    int detection_index, KeypointsStamped &output) const;
};
}  // namespace auto_battlebot
