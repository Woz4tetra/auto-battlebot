#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "config/enum_map_config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "enums/keypoint_label.hpp"
#include "enums/label.hpp"
#include "keypoint_model/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "label_utils.hpp"
#include "tensorrt_inference/trt_engine.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
// One detection row: [x1, y1, x2, y2, conf, class_id, kp1_x, kp1_y, kp1_conf, ...]
using DetectionRow = std::vector<float>;

class YoloKeypointModel : public KeypointModelInterface {
   public:
    explicit YoloKeypointModel(YoloKeypointModelConfiguration &config);

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
    std::vector<Label> label_indices_;

    TrtEngine engine_;
    bool initialized_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    // Helper methods
    float generate_scale(cv::Mat &image, const std::vector<int> &target_size);
    float letterbox(cv::Mat &input_image, cv::Mat &output_image,
                    const std::vector<int> &target_size);
    void preprocess_image(const cv::Mat &image, cv::Size input_image_size,
                          std::vector<float> &buffer);

    std::vector<int64_t> nms(const float *bboxes, const float *scores, int64_t ndets,
                             float iou_threshold);
    std::vector<DetectionRow> non_max_suppression(const float *prediction, int num_features,
                                                  int num_predictions, int num_keypoints,
                                                  float conf_thres, float iou_thres, int max_det);
    void xywh2xyxy(float *boxes, int64_t n);

    Keypoint scale_keypoint(Keypoint output_keypoint, cv::Size original_image_size,
                            cv::Size input_image_size);
    void scale_boxes(std::vector<DetectionRow> &detections, cv::Size original_image_size,
                     cv::Size input_image_size);

    KeypointsStamped postprocess_output(const float *output, const Header &header,
                                        cv::Size original_image_size, cv::Size input_image_size,
                                        const cv::Mat &original_image);
    void visualize_output(const cv::Mat &original_image, const KeypointsStamped &keypoints,
                          const std::vector<DetectionRow> &detections);
};

}  // namespace auto_battlebot
