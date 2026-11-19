#pragma once

#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <magic_enum.hpp>
#include <opencv2/opencv.hpp>
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/enum_map_config.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "engine_selector/engine_selector.hpp"
#include "enums/keypoint_label.hpp"
#include "enums/label.hpp"
#include "keypoint_model/config.hpp"
#include "keypoint_model/keypoint_model_interface.hpp"
#include "label_utils.hpp"
#include "tensorrt_inference/gpu_letterbox.hpp"
#include "tensorrt_inference/trt_engine.hpp"
#include "time_utils.hpp"

namespace auto_battlebot {
using DetectionRow = std::vector<float>;
using BoxXyxy = std::array<float, 4>;

struct PredictionMatrix {
    std::span<const float> values;
    int num_predictions = 0;
    int num_features = 0;

    float at(int prediction, int feature) const {
        return values[static_cast<size_t>(prediction) * num_features + feature];
    }
};

namespace detection_row {
constexpr size_t kX1 = 0;
constexpr size_t kY1 = 1;
constexpr size_t kX2 = 2;
constexpr size_t kY2 = 3;
constexpr size_t kConfidence = 4;
constexpr size_t kClassId = 5;
constexpr size_t kFirstKeypoint = 6;
constexpr size_t kKeypointStride = 3;
constexpr size_t kKeypointYOffset = 1;
constexpr size_t kKeypointConfidenceOffset = 2;
}  // namespace detection_row

std::vector<BoundingBox> boxes_per_row(const std::vector<DetectionRow> &rows);

class YoloKeypointModel : public KeypointModelInterface {
   public:
    YoloKeypointModel(YoloKeypointModelConfiguration &config,
                      std::shared_ptr<EngineSelector> engine_selector);

    bool initialize() override;
    ModelResultStamped update(RgbImage image, const CameraInfo &camera_info) override;
    DetectionsStamped last_detections() const override { return last_detections_; }

   protected:
    std::string model_path_;
    float threshold_;
    float iou_threshold_;
    float letterbox_padding_;
    int image_size_;
    bool debug_visualization_;
    LabelToKeypointMapConfiguration label_map_;
    std::vector<Label> label_indices_;
    std::unordered_map<Label, double> reference_offset_m_per_label_;

    TrtEngine engine_;
    GpuLetterbox letterbox_;
    std::shared_ptr<EngineSelector> engine_selector_;
    bool initialized_;
    DetectionsStamped last_detections_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    void record_raw_detections(const std::vector<DetectionRow> &keep, cv::Size original_image_size,
                               cv::Size input_image_size);

    std::vector<int64_t> nms(std::span<const BoxXyxy> bboxes, std::span<const float> scores,
                             float iou_threshold);
    std::vector<DetectionRow> non_max_suppression(const PredictionMatrix &prediction,
                                                  int num_keypoints, float conf_thres,
                                                  float iou_thres, int max_det);
    void xywh2xyxy(float *boxes, int64_t n);

    Keypoint scale_keypoint(Keypoint output_keypoint, cv::Size original_image_size,
                            cv::Size input_image_size);
    void scale_boxes(std::vector<DetectionRow> &detections, cv::Size original_image_size,
                     cv::Size input_image_size);

    bool resolve_output_layout(const float *output, int &num_features, int &num_predictions,
                               std::vector<float> &transposed_buf, const float *&prediction_ptr);

    int extract_keypoints_from_detections(const std::vector<DetectionRow> &keep,
                                          const Header &header, cv::Size original_image_size,
                                          cv::Size input_image_size, ModelResultStamped &result);

    ModelResultStamped postprocess_output(const float *output, const Header &header,
                                          cv::Size original_image_size, cv::Size input_image_size,
                                          const cv::Mat &original_image);
    void visualize_output(const cv::Mat &original_image, const ModelResultStamped &model_results,
                          const std::vector<DetectionRow> &detections);
};

}  // namespace auto_battlebot
