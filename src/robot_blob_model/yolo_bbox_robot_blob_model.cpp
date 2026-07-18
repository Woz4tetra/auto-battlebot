#include "robot_blob_model/yolo_bbox_robot_blob_model.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#include "enum_to_string_lower.hpp"

namespace auto_battlebot {
namespace {
constexpr float kMinBoxEdge = 1.0f;
}  // namespace

YoloBboxRobotBlobModel::YoloBboxRobotBlobModel(YoloBboxRobotBlobModelConfiguration &config)
    : model_path_(config.model_path),
      confidence_threshold_(config.confidence_threshold),
      iou_threshold_(config.iou_threshold),
      letterbox_padding_(config.letterbox_padding),
      image_size_(config.image_size),
      max_detections_(config.max_detections),
      debug_visualization_(config.debug_visualization),
      label_indices_(config.label_indices),
      their_robot_labels_(config.their_robot_labels.begin(), config.their_robot_labels.end()),
      neutral_robot_labels_(config.neutral_robot_labels.begin(), config.neutral_robot_labels.end()),
      field_labels_(config.field_labels.begin(), config.field_labels.end()) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("yolo_bbox_robot_blob_model");
}

bool YoloBboxRobotBlobModel::initialize() {
    spdlog::info("Loading YOLO-bbox TensorRT engine from: {}", model_path_);
    if (!engine_.load(model_path_)) {
        spdlog::error("Failed to load YOLO-bbox TensorRT engine: {}", model_path_);
        return false;
    }

    std::vector<float> warmup_input(static_cast<size_t>(engine_.getInputNumElements()), 0.0f);
    std::vector<float> warmup_output(static_cast<size_t>(engine_.getOutputNumElements()), 0.0f);
    for (int i = 0; i < 2; ++i) {
        if (!engine_.execute(warmup_input.data(), warmup_output.data())) {
            spdlog::error("YOLO-bbox warmup inference failed");
            return false;
        }
    }
    spdlog::info("YoloBboxRobotBlobModel initialized");

    initialized_ = true;
    diagnostics_logger_->info({}, "YoloBboxRobotBlobModel initialized successfully");
    return true;
}

KeypointsStamped YoloBboxRobotBlobModel::update(RgbImage image) {
    FunctionTimer timer(diagnostics_logger_, "update", 1000.0);
    KeypointsStamped result;
    result.header = image.header;
    last_detections_ = DetectionsStamped{};
    last_detections_.header = image.header;
    last_detections_.image_width = image.image.cols;
    last_detections_.image_height = image.image.rows;
    if (!initialized_) {
        diagnostics_logger_->error({}, "Model not initialized");
        return result;
    }

    const std::vector<int64_t> in_shape = engine_.getInputShape();
    if (in_shape.size() < 4 || in_shape[0] != 1 || in_shape[1] != 3) {
        diagnostics_logger_->error({}, "YOLO-bbox input shape invalid");
        return result;
    }
    const cv::Size input_size(static_cast<int>(in_shape[3]), static_cast<int>(in_shape[2]));
    const cv::Size original_size(image.image.cols, image.image.rows);

    std::vector<float> input_buffer;
    preprocess_image(image.image, input_size, input_buffer);
    if (static_cast<int64_t>(input_buffer.size()) != engine_.getInputNumElements()) {
        diagnostics_logger_->error({}, "YOLO-bbox input buffer size mismatch");
        return result;
    }

    const std::vector<int64_t> out_shape = engine_.getOutputShape();
    std::vector<float> output_buffer(static_cast<size_t>(engine_.getOutputNumElements()), 0.0f);
    if (!engine_.execute(input_buffer.data(), output_buffer.data())) {
        diagnostics_logger_->error({}, "YOLO-bbox inference failed");
        return result;
    }

    auto detections = decode_detections(output_buffer, out_shape);
    detections = non_max_suppression(detections);
    if (static_cast<int>(detections.size()) > max_detections_) {
        detections.resize(static_cast<size_t>(max_detections_));
    }

    cv::Mat debug_vis;
    const bool render_debug = debug_visualization_ && !image.image.empty();
    if (render_debug) {
        debug_vis = image.image.clone();
    }

    int detection_index = 0;
    for (const auto &det : detections) {
        // Log the raw class before it is collapsed to OPPONENT/HOUSE_BOT (or dropped),
        // so the mislabel/drop rate is visible in future recordings: e.g. an opponent
        // classified as one of our robots falls to OTHER and is never targeted.
        if (det.class_id >= 0 && det.class_id < static_cast<int>(label_indices_.size())) {
            const Label raw_label = label_indices_[static_cast<size_t>(det.class_id)];
            diagnostics_logger_->info(
                "detection-" + std::to_string(detection_index),
                {{"raw_class", enum_to_string_lower(raw_label)},
                 {"category", enum_to_string_lower(classify_category(raw_label))},
                 {"confidence", static_cast<double>(det.confidence)}});

            // Keep the raw box (original-image pixels) so offline eval tools can score
            // the detector itself, independent of the blob/keypoint collapse below.
            int box_x1 = 0;
            int box_y1 = 0;
            int box_x2 = 0;
            int box_y2 = 0;
            map_detection_box_to_original(det, original_size, input_size, box_x1, box_y1, box_x2,
                                          box_y2);
            last_detections_.detections.push_back(Detection2D{static_cast<double>(box_x1),
                                                              static_cast<double>(box_y1),
                                                              static_cast<double>(box_x2),
                                                              static_cast<double>(box_y2),
                                                              static_cast<double>(det.confidence),
                                                              det.class_id,
                                                              raw_label,
                                                              {}});
        }

        if (render_debug) {
            render_detection_debug(debug_vis, det, original_size, input_size);
        }

        append_detection_keypoints(det, original_size, input_size, detection_index, result);
        ++detection_index;
    }

    if (render_debug) {
        render_keypoints_debug(debug_vis, result);
        cv::imshow("YOLO Bbox Robot Blob Debug", debug_vis);
        cv::waitKey(1);
    }

    return result;
}

void YoloBboxRobotBlobModel::map_detection_box_to_original(const Detection &det,
                                                           cv::Size original_size,
                                                           cv::Size input_size, int &x1, int &y1,
                                                           int &x2, int &y2) const {
    const double gain = std::min(static_cast<double>(input_size.width) / original_size.width,
                                 static_cast<double>(input_size.height) / original_size.height);
    const double pad_w =
        std::round((input_size.width - original_size.width * gain) / 2.0 - letterbox_padding_);
    const double pad_h =
        std::round((input_size.height - original_size.height * gain) / 2.0 - letterbox_padding_);
    x1 = static_cast<int>(std::round((det.x1 - pad_w) / gain));
    y1 = static_cast<int>(std::round((det.y1 - pad_h) / gain));
    x2 = static_cast<int>(std::round((det.x2 - pad_w) / gain));
    y2 = static_cast<int>(std::round((det.y2 - pad_h) / gain));
}

cv::Scalar YoloBboxRobotBlobModel::debug_color_for_detection(const Detection &det) const {
    if (det.class_id < 0 || det.class_id >= static_cast<int>(label_indices_.size())) {
        return cv::Scalar(180, 180, 180);
    }
    const Label cls = label_indices_[static_cast<size_t>(det.class_id)];
    const Category cat = classify_category(cls);
    if (cat == Category::THEIR) return cv::Scalar(0, 0, 255);
    if (cat == Category::NEUTRAL) return cv::Scalar(255, 255, 0);
    if (cat == Category::FIELD) return cv::Scalar(0, 255, 0);
    return cv::Scalar(180, 180, 180);
}

void YoloBboxRobotBlobModel::render_detection_debug(cv::Mat &debug_vis, const Detection &det,
                                                    cv::Size original_size,
                                                    cv::Size input_size) const {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    map_detection_box_to_original(det, original_size, input_size, x1, y1, x2, y2);
    const cv::Scalar color = debug_color_for_detection(det);

    cv::rectangle(debug_vis, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
    const std::string det_text =
        "cls=" + std::to_string(det.class_id) + " conf=" + cv::format("%.2f", det.confidence);
    cv::putText(debug_vis, det_text, cv::Point(x1, std::max(0, y1 - 6)), cv::FONT_HERSHEY_SIMPLEX,
                0.5, color, 1);
}

void YoloBboxRobotBlobModel::render_keypoints_debug(cv::Mat &debug_vis,
                                                    const KeypointsStamped &keypoints) {
    for (const auto &kp : keypoints.keypoints) {
        const int x = static_cast<int>(std::round(kp.x));
        const int y = static_cast<int>(std::round(kp.y));
        cv::circle(debug_vis, cv::Point(x, y), 5, cv::Scalar(255, 255, 255), -1);
        cv::circle(debug_vis, cv::Point(x, y), 3, cv::Scalar(0, 0, 0), -1);
    }
}

void YoloBboxRobotBlobModel::preprocess_image(const cv::Mat &image, cv::Size input_size,
                                              std::vector<float> &buffer) {
    cv::Mat processed_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, processed_image, cv::COLOR_BGR2RGB);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, processed_image, cv::COLOR_BGRA2RGB);
    } else {
        processed_image = image;
    }

    cv::Mat resized;
    letterbox(processed_image, resized, {input_size.height, input_size.width});

    cv::Mat float_image;
    resized.convertTo(float_image, CV_32F, 1.0 / 255.0);
    if (!float_image.isContinuous()) float_image = float_image.clone();

    const int H = float_image.rows;
    const int W = float_image.cols;
    buffer.resize(static_cast<size_t>(3 * H * W));
    float *ptr = buffer.data();
    for (int c = 0; c < 3; ++c) {
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                ptr[c * H * W + y * W + x] = float_image.at<cv::Vec3f>(y, x)[c];
            }
        }
    }
}

float YoloBboxRobotBlobModel::generate_scale(cv::Mat &image, const std::vector<int> &target_size) {
    const float ratio_h = static_cast<float>(target_size[0]) / static_cast<float>(image.rows);
    const float ratio_w = static_cast<float>(target_size[1]) / static_cast<float>(image.cols);
    return std::min(ratio_h, ratio_w);
}

float YoloBboxRobotBlobModel::letterbox(cv::Mat &input_image, cv::Mat &output_image,
                                        const std::vector<int> &target_size) {
    if (input_image.cols == target_size[1] && input_image.rows == target_size[0]) {
        output_image = input_image.clone();
        return 1.0f;
    }
    const float resize_scale = generate_scale(input_image, target_size);
    const int new_w = static_cast<int>(std::round(input_image.cols * resize_scale));
    const int new_h = static_cast<int>(std::round(input_image.rows * resize_scale));
    const float padw = (target_size[1] - new_w) / 2.0f;
    const float padh = (target_size[0] - new_h) / 2.0f;
    const int top = static_cast<int>(std::round(padh - letterbox_padding_));
    const int bottom = static_cast<int>(std::round(padh + letterbox_padding_));
    const int left = static_cast<int>(std::round(padw - letterbox_padding_));
    const int right = static_cast<int>(std::round(padw + letterbox_padding_));

    cv::resize(input_image, output_image, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::copyMakeBorder(output_image, output_image, top, bottom, left, right, cv::BORDER_CONSTANT,
                       cv::Scalar(114, 114, 114));
    return resize_scale;
}

std::vector<YoloBboxRobotBlobModel::Detection> YoloBboxRobotBlobModel::decode_detections(
    const std::vector<float> &det_output, const std::vector<int64_t> &det_shape) const {
    std::vector<Detection> detections;
    if (det_shape.size() < 3) return detections;

    const int feature_dim = static_cast<int>(det_shape[1]);
    const int num_predictions = static_cast<int>(det_shape[2]);
    const int class_dim = static_cast<int>(label_indices_.size());
    // Detect head is [1, 4 + num_classes, num_predictions]: 4 box coords then per-class scores,
    // with no trailing mask coefficients (unlike the seg head).
    if (feature_dim < 4 + class_dim) return detections;

    auto read_feature = [&](int prediction_index, int feature_index) -> float {
        return det_output[static_cast<size_t>(feature_index) * num_predictions + prediction_index];
    };

    for (int i = 0; i < num_predictions; ++i) {
        const float cx = read_feature(i, 0);
        const float cy = read_feature(i, 1);
        const float w = read_feature(i, 2);
        const float h = read_feature(i, 3);
        if (w < kMinBoxEdge || h < kMinBoxEdge) continue;

        float best_score = -1.0f;
        int best_class = 0;
        for (int c = 0; c < class_dim; ++c) {
            const float score = read_feature(i, 4 + c);
            if (score > best_score) {
                best_score = score;
                best_class = c;
            }
        }
        if (best_score < confidence_threshold_) continue;

        Detection det;
        det.x1 = cx - 0.5f * w;
        det.y1 = cy - 0.5f * h;
        det.x2 = cx + 0.5f * w;
        det.y2 = cy + 0.5f * h;
        det.confidence = best_score;
        det.class_id = best_class;
        detections.push_back(det);
    }
    return detections;
}

std::vector<YoloBboxRobotBlobModel::Detection> YoloBboxRobotBlobModel::non_max_suppression(
    const std::vector<Detection> &detections) const {
    std::vector<Detection> ordered = detections;
    std::sort(ordered.begin(), ordered.end(), [](const Detection &lhs, const Detection &rhs) {
        return lhs.confidence > rhs.confidence;
    });

    std::vector<Detection> kept;
    std::vector<bool> suppressed(ordered.size(), false);
    for (size_t i = 0; i < ordered.size(); ++i) {
        if (suppressed[i]) continue;
        kept.push_back(ordered[i]);
        for (size_t j = i + 1; j < ordered.size(); ++j) {
            if (suppressed[j]) continue;
            if (ordered[i].class_id != ordered[j].class_id) continue;
            if (iou(ordered[i], ordered[j]) > iou_threshold_) {
                suppressed[j] = true;
            }
        }
    }
    return kept;
}

float YoloBboxRobotBlobModel::iou(const Detection &lhs, const Detection &rhs) {
    const float x1 = std::max(lhs.x1, rhs.x1);
    const float y1 = std::max(lhs.y1, rhs.y1);
    const float x2 = std::min(lhs.x2, rhs.x2);
    const float y2 = std::min(lhs.y2, rhs.y2);
    const float w = std::max(0.0f, x2 - x1);
    const float h = std::max(0.0f, y2 - y1);
    const float inter = w * h;
    const float area_l = std::max(0.0f, lhs.x2 - lhs.x1) * std::max(0.0f, lhs.y2 - lhs.y1);
    const float area_r = std::max(0.0f, rhs.x2 - rhs.x1) * std::max(0.0f, rhs.y2 - rhs.y1);
    const float denom = area_l + area_r - inter;
    if (denom <= 0.0f) return 0.0f;
    return inter / denom;
}

YoloBboxRobotBlobModel::Category YoloBboxRobotBlobModel::classify_category(Label label) const {
    if (their_robot_labels_.count(label) > 0) return Category::THEIR;
    if (neutral_robot_labels_.count(label) > 0) return Category::NEUTRAL;
    if (field_labels_.count(label) > 0) return Category::FIELD;
    return Category::OTHER;
}

void YoloBboxRobotBlobModel::append_detection_keypoints(const Detection &det,
                                                        cv::Size original_size, cv::Size input_size,
                                                        int detection_index,
                                                        KeypointsStamped &output) const {
    if (det.class_id < 0 || det.class_id >= static_cast<int>(label_indices_.size())) return;
    const Label class_label = label_indices_[static_cast<size_t>(det.class_id)];
    const Category category = classify_category(class_label);
    if (category != Category::THEIR && category != Category::NEUTRAL) return;

    const Label output_label = (category == Category::THEIR) ? Label::OPPONENT : Label::HOUSE_BOT;
    const KeypointLabel a_label = (category == Category::THEIR) ? KeypointLabel::OPPONENT_FRONT
                                                                : KeypointLabel::HOUSE_BOT_FRONT;
    const KeypointLabel b_label = (category == Category::THEIR) ? KeypointLabel::OPPONENT_BACK
                                                                : KeypointLabel::HOUSE_BOT_BACK;

    // No mask to fit a rotated rectangle to, so the midline comes straight from the
    // detection box (long axis). Downstream target selection only needs the box center
    // and a front/back axis, both of which the box supplies.
    const double gain = std::min(static_cast<double>(input_size.width) / original_size.width,
                                 static_cast<double>(input_size.height) / original_size.height);
    const double pad_w =
        std::round((input_size.width - original_size.width * gain) / 2.0 - letterbox_padding_);
    const double pad_h =
        std::round((input_size.height - original_size.height * gain) / 2.0 - letterbox_padding_);
    const float x1 = static_cast<float>((det.x1 - pad_w) / gain);
    const float y1 = static_cast<float>((det.y1 - pad_h) / gain);
    const float x2 = static_cast<float>((det.x2 - pad_w) / gain);
    const float y2 = static_cast<float>((det.y2 - pad_h) / gain);
    const MidlineSegment midline = bbox_to_midline(x1, y1, x2, y2);
    append_midline_keypoints(midline, output_label, a_label, b_label, det.confidence,
                             detection_index, output.keypoints);
}
}  // namespace auto_battlebot
