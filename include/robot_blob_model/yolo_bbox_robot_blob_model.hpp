#pragma once

#include <set>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"
#include "engine_selector/engine_selector.hpp"
#include "robot_blob_model/config.hpp"
#include "robot_blob_model/rectangle_keypoint_helpers.hpp"
#include "robot_blob_model/robot_blob_model_interface.hpp"
#include "tensorrt_inference/trt_engine.hpp"

namespace auto_battlebot {
// Bounding-box-only variant of YoloSegRobotBlobModel. Runs a plain YOLO detect
// engine (output [1, 4 + num_classes, num_predictions]) with no mask-prototype
// branch, so it is narrower and faster on the same images. Downstream consumers
// only need box centers and the front/back midline, both of which come straight
// from the box, so dropping the segmentation head costs nothing they read.
// See docs/experiments/perception_performance/seg_vs_bbox_2026-07-18.md.
class YoloBboxRobotBlobModel : public RobotBlobModelInterface {
   public:
    YoloBboxRobotBlobModel(YoloBboxRobotBlobModelConfiguration &config,
                           std::shared_ptr<EngineSelector> engine_selector);

    bool initialize() override;
    KeypointsStamped update(RgbImage image) override;
    DetectionsStamped last_detections() const override { return last_detections_; }

   private:
    struct Detection {
        float x1 = 0.0f;
        float y1 = 0.0f;
        float x2 = 0.0f;
        float y2 = 0.0f;
        float confidence = 0.0f;
        int class_id = 0;
    };

    enum class Category { THEIR, NEUTRAL, FIELD, OTHER };

    std::string model_path_;
    float confidence_threshold_;
    float iou_threshold_;
    float letterbox_padding_;
    int image_size_;
    int max_detections_;
    bool debug_visualization_;
    std::vector<Label> label_indices_;
    std::set<Label> their_robot_labels_;
    std::set<Label> neutral_robot_labels_;
    std::set<Label> field_labels_;

    TrtEngine engine_;
    std::shared_ptr<EngineSelector> engine_selector_;
    bool initialized_ = false;
    DetectionsStamped last_detections_;
    std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

    void preprocess_image(const cv::Mat &image, cv::Size input_size, std::vector<float> &buffer);
    float letterbox(cv::Mat &input_image, cv::Mat &output_image,
                    const std::vector<int> &target_size);
    float generate_scale(cv::Mat &image, const std::vector<int> &target_size);

    std::vector<Detection> decode_detections(const std::vector<float> &det_output,
                                             const std::vector<int64_t> &det_shape) const;
    std::vector<Detection> non_max_suppression(const std::vector<Detection> &detections) const;
    static float iou(const Detection &lhs, const Detection &rhs);

    void map_detection_box_to_original(const Detection &det, cv::Size original_size,
                                       cv::Size input_size, int &x1, int &y1, int &x2,
                                       int &y2) const;
    cv::Scalar debug_color_for_detection(const Detection &det) const;
    void render_detection_debug(cv::Mat &debug_vis, const Detection &det, cv::Size original_size,
                                cv::Size input_size) const;
    static void render_keypoints_debug(cv::Mat &debug_vis, const KeypointsStamped &keypoints);
    Category classify_category(Label label) const;

    void append_detection_keypoints(const Detection &det, cv::Size original_size,
                                    cv::Size input_size, int detection_index,
                                    KeypointsStamped &output) const;
};
}  // namespace auto_battlebot
