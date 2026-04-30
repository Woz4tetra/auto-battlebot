#include "keypoint_model/yolo_keypoint_model.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace auto_battlebot {
YoloKeypointModel::YoloKeypointModel(YoloKeypointModelConfiguration &config)
    : model_path_(config.model_path),
      threshold_(config.threshold),
      iou_threshold_(config.iou_threshold),
      letterbox_padding_(config.letterbox_padding),
      image_size_(config.image_size),
      debug_visualization_(config.debug_visualization),
      label_map_(config.label_map),
      label_indices_(config.label_indices),
      initialized_(false),
      diagnostics_logger_(DiagnosticsLogger::get_logger("yolo_keypoint_model")) {}

bool YoloKeypointModel::initialize() {
    spdlog::info("Loading TensorRT engine from: {}", model_path_);
    if (!engine_.load(model_path_)) {
        spdlog::error("Failed to load YOLO TensorRT engine: {}", model_path_);
        return false;
    }

    spdlog::info("Warming up model with dummy input...");
    std::vector<float> warmup_input(static_cast<size_t>(engine_.getInputNumElements()), 0.0f);
    std::vector<float> warmup_output(static_cast<size_t>(engine_.getOutputNumElements()), 0.0f);
    for (int i = 0; i < 3; i++) {
        if (!engine_.execute(warmup_input.data(), warmup_output.data())) {
            spdlog::error("YOLO warmup inference failed");
            return false;
        }
    }
    spdlog::info("YoloKeypointModel initialized");

    initialized_ = true;
    diagnostics_logger_->info({}, "YoloKeypointModel initialized successfully");
    return true;
}

KeypointsStamped YoloKeypointModel::update(RgbImage image) {
    FunctionTimer timer(diagnostics_logger_, "update", 1000.0);

    if (!initialized_) {
        diagnostics_logger_->error({}, "Model not initialized");
        return KeypointsStamped{};
    }
    const std::vector<int64_t> in_shape = engine_.getInputShape();
    if (in_shape.size() < 4 || in_shape[0] != 1 || in_shape[1] != 3) {
        diagnostics_logger_->error({}, "YOLO engine input shape invalid");
        return KeypointsStamped{};
    }
    const cv::Size input_image_size(static_cast<int>(in_shape[3]), static_cast<int>(in_shape[2]));
    const cv::Size original_image_size(image.image.cols, image.image.rows);

    std::vector<float> input_buffer;
    preprocess_image(image.image, input_image_size, input_buffer);
    if (static_cast<int64_t>(input_buffer.size()) != engine_.getInputNumElements()) {
        diagnostics_logger_->error({}, "YOLO input buffer size mismatch");
        return KeypointsStamped{};
    }

    std::vector<float> output_buffer(static_cast<size_t>(engine_.getOutputNumElements()));
    if (!engine_.execute(input_buffer.data(), output_buffer.data())) {
        diagnostics_logger_->error({}, "YOLO inference failed");
        return KeypointsStamped{};
    }

    return postprocess_output(output_buffer.data(), image.header, original_image_size,
                              input_image_size, image.image);
}

void YoloKeypointModel::preprocess_image(const cv::Mat &image, cv::Size input_image_size,
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
    letterbox(processed_image, resized, {input_image_size.height, input_image_size.width});

    cv::Mat float_image;
    resized.convertTo(float_image, CV_32F, 1.0 / 255.0);
    if (!float_image.isContinuous()) {
        float_image = float_image.clone();
    }

    const int H = float_image.rows;
    const int W = float_image.cols;
    const int64_t num_elements = 1 * 3 * H * W;
    buffer.resize(static_cast<size_t>(num_elements));
    float *ptr = buffer.data();
    for (int c = 0; c < 3; c++) {
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                ptr[c * H * W + y * W + x] = float_image.at<cv::Vec3f>(y, x)[c];
            }
        }
    }
}

float YoloKeypointModel::generate_scale(cv::Mat &image, const std::vector<int> &target_size) {
    const int origin_w = image.cols;
    const int origin_h = image.rows;
    const int target_h = target_size[0];
    const int target_w = target_size[1];
    const float ratio_h = static_cast<float>(target_h) / static_cast<float>(origin_h);
    const float ratio_w = static_cast<float>(target_w) / static_cast<float>(origin_w);
    return std::min(ratio_h, ratio_w);
}

float YoloKeypointModel::letterbox(cv::Mat &input_image, cv::Mat &output_image,
                                   const std::vector<int> &target_size) {
    if (input_image.cols == target_size[1] && input_image.rows == target_size[0]) {
        if (input_image.data == output_image.data) {
            return 1.f;
        }
        output_image = input_image.clone();
        return 1.f;
    }

    const float resize_scale = generate_scale(input_image, target_size);
    const int new_shape_w = static_cast<int>(std::round(input_image.cols * resize_scale));
    const int new_shape_h = static_cast<int>(std::round(input_image.rows * resize_scale));
    const float padw = (target_size[1] - new_shape_w) / 2.0f;
    const float padh = (target_size[0] - new_shape_h) / 2.0f;

    const int top = static_cast<int>(std::round(padh - letterbox_padding_));
    const int bottom = static_cast<int>(std::round(padh + letterbox_padding_));
    const int left = static_cast<int>(std::round(padw - letterbox_padding_));
    const int right = static_cast<int>(std::round(padw + letterbox_padding_));

    cv::resize(input_image, output_image, cv::Size(new_shape_w, new_shape_h), 0, 0,
               cv::INTER_LINEAR);
    cv::copyMakeBorder(output_image, output_image, top, bottom, left, right, cv::BORDER_CONSTANT,
                       cv::Scalar(114.0, 114.0, 114.0));
    return resize_scale;
}

// NMS: bboxes is ndets x 4 (x1,y1,x2,y2), scores is ndets. Returns indices to keep.
std::vector<int64_t> YoloKeypointModel::nms(const float *bboxes, const float *scores, int64_t ndets,
                                            float iou_threshold) {
    std::vector<int64_t> keep;
    if (ndets == 0) return keep;

    std::vector<float> areas(static_cast<size_t>(ndets));
    for (int64_t i = 0; i < ndets; i++) {
        const float x1 = bboxes[i * 4 + 0];
        const float y1 = bboxes[i * 4 + 1];
        const float x2 = bboxes[i * 4 + 2];
        const float y2 = bboxes[i * 4 + 3];
        areas[i] = (x2 - x1) * (y2 - y1);
    }

    std::vector<int64_t> order(static_cast<size_t>(ndets));
    for (int64_t i = 0; i < ndets; i++) order[i] = i;
    std::stable_sort(order.begin(), order.end(),
                     [&scores](int64_t a, int64_t b) { return scores[a] > scores[b]; });

    std::vector<uint8_t> suppressed(static_cast<size_t>(ndets), 0);

    for (int64_t _i = 0; _i < ndets; _i++) {
        const int64_t i = order[_i];
        if (suppressed[i]) continue;
        keep.push_back(i);
        const float ix1 = bboxes[i * 4 + 0];
        const float iy1 = bboxes[i * 4 + 1];
        const float ix2 = bboxes[i * 4 + 2];
        const float iy2 = bboxes[i * 4 + 3];
        const float iarea = areas[i];

        for (int64_t _j = _i + 1; _j < ndets; _j++) {
            const int64_t j = order[_j];
            if (suppressed[j]) continue;
            const float xx1 = std::max(ix1, bboxes[j * 4 + 0]);
            const float yy1 = std::max(iy1, bboxes[j * 4 + 1]);
            const float xx2 = std::min(ix2, bboxes[j * 4 + 2]);
            const float yy2 = std::min(iy2, bboxes[j * 4 + 3]);
            const float w = std::max(0.f, xx2 - xx1);
            const float h = std::max(0.f, yy2 - yy1);
            const float inter = w * h;
            const float ovr = inter / (iarea + areas[j] - inter);
            if (ovr > iou_threshold) suppressed[j] = 1;
        }
    }
    return keep;
}

void YoloKeypointModel::xywh2xyxy(float *boxes, int64_t n) {
    for (int64_t i = 0; i < n; i++) {
        float *b = boxes + i * 4;
        const float dw = b[2] / 2.f;
        const float dh = b[3] / 2.f;
        const float cx = b[0];
        const float cy = b[1];
        b[0] = cx - dw;
        b[1] = cy - dh;
        b[2] = cx + dw;
        b[3] = cy + dh;
    }
}

// prediction is [num_predictions, num_features] row-major (caller normalizes layout to match
// Python). Bbox format is (x1, y1, x2, y2) per row.
std::vector<DetectionRow> YoloKeypointModel::non_max_suppression(
    const float *prediction, int num_features, int num_predictions, int num_keypoints,
    float conf_thres, float iou_thres, int max_det) {
    const int num_keypoint_values = num_keypoints * 3;
    const int num_classes = num_features - 4 - num_keypoint_values;
    const int class_score_end = 4 + num_classes;
    if (num_classes <= 0 || num_predictions <= 0) return {};

    std::vector<int> class_conf_mask(static_cast<size_t>(num_predictions));
    for (int p = 0; p < num_predictions; p++) {
        float max_cls = prediction[p * num_features + 4];
        for (int c = 1; c < num_classes; c++) {
            const float v = prediction[p * num_features + 4 + c];
            if (v > max_cls) max_cls = v;
        }
        class_conf_mask[p] = (max_cls > conf_thres) ? 1 : 0;
    }

    std::vector<float> boxes_xyxy(static_cast<size_t>(num_predictions) * 4);
    for (int p = 0; p < num_predictions; p++) {
        for (int j = 0; j < 4; j++) boxes_xyxy[p * 4 + j] = prediction[p * num_features + j];
    }
    xywh2xyxy(boxes_xyxy.data(), num_predictions);

    std::vector<DetectionRow> out_rows;
    std::vector<float> det_boxes(static_cast<size_t>(num_predictions) * 4);
    std::vector<float> det_scores(static_cast<size_t>(num_predictions));

    int det_count = 0;
    for (int p = 0; p < num_predictions; p++) {
        if (!class_conf_mask[p]) continue;
        float max_conf = prediction[p * num_features + 4];
        int best_cls = 0;
        for (int c = 1; c < num_classes; c++) {
            const float v = prediction[p * num_features + 4 + c];
            if (v > max_conf) {
                max_conf = v;
                best_cls = c;
            }
        }
        if (max_conf < conf_thres) continue;

        for (int j = 0; j < 4; j++) det_boxes[det_count * 4 + j] = boxes_xyxy[p * 4 + j];
        det_scores[det_count] = max_conf;

        DetectionRow row;
        row.reserve(static_cast<size_t>(6 + num_keypoint_values));
        row.push_back(boxes_xyxy[p * 4 + 0]);
        row.push_back(boxes_xyxy[p * 4 + 1]);
        row.push_back(boxes_xyxy[p * 4 + 2]);
        row.push_back(boxes_xyxy[p * 4 + 3]);
        row.push_back(max_conf);
        row.push_back(static_cast<float>(best_cls));
        for (int k = 0; k < num_keypoint_values; k++) {
            float val = prediction[p * num_features + class_score_end + k];
            row.push_back(val);
        }
        out_rows.push_back(std::move(row));
        det_count++;
    }

    if (out_rows.empty()) return out_rows;

    det_boxes.resize(static_cast<size_t>(det_count) * 4);
    det_scores.resize(static_cast<size_t>(det_count));

    std::vector<float> nms_boxes(static_cast<size_t>(det_count) * 4);
    for (int64_t i = 0; i < det_count; i++) {
        const float class_offset = out_rows[i][5] * 7680.f;
        nms_boxes[i * 4 + 0] = out_rows[i][0] + class_offset;
        nms_boxes[i * 4 + 1] = out_rows[i][1] + class_offset;
        nms_boxes[i * 4 + 2] = out_rows[i][2] + class_offset;
        nms_boxes[i * 4 + 3] = out_rows[i][3] + class_offset;
    }
    std::vector<float> nms_scores(static_cast<size_t>(det_count));
    for (int64_t i = 0; i < det_count; i++) nms_scores[i] = out_rows[i][4];

    std::vector<int64_t> keep = nms(nms_boxes.data(), nms_scores.data(), det_count, iou_thres);
    if (static_cast<int>(keep.size()) > max_det) keep.resize(static_cast<size_t>(max_det));

    std::vector<DetectionRow> result;
    result.reserve(keep.size());
    for (int64_t k : keep) result.push_back(out_rows[k]);
    return result;
}

Keypoint YoloKeypointModel::scale_keypoint(Keypoint output_keypoint, cv::Size original_image_size,
                                           cv::Size input_image_size) {
    const double gain =
        std::min(static_cast<double>(input_image_size.width) / original_image_size.width,
                 static_cast<double>(input_image_size.height) / original_image_size.height);
    const double pad0 = std::round(
        (input_image_size.width - original_image_size.width * gain) / 2.0 - letterbox_padding_);
    const double pad1 = std::round(
        (input_image_size.height - original_image_size.height * gain) / 2.0 - letterbox_padding_);
    double x = output_keypoint.x;
    double y = output_keypoint.y;
    x -= pad0;
    y -= pad1;
    x /= gain;
    y /= gain;
    return Keypoint{output_keypoint.label, output_keypoint.keypoint_label, x, y};
}

void YoloKeypointModel::scale_boxes(std::vector<DetectionRow> &detections,
                                    cv::Size original_image_size, cv::Size input_image_size) {
    const float gain =
        std::min(static_cast<float>(input_image_size.height) / original_image_size.height,
                 static_cast<float>(input_image_size.width) / original_image_size.width);
    const float pad0 = static_cast<float>(std::round(
        (input_image_size.width - original_image_size.width * gain) / 2.0 - letterbox_padding_));
    const float pad1 = static_cast<float>(std::round(
        (input_image_size.height - original_image_size.height * gain) / 2.0 - letterbox_padding_));
    for (auto &row : detections) {
        if (row.size() < 4) continue;
        row[0] = (row[0] - pad0) / gain;
        row[2] = (row[2] - pad0) / gain;
        row[1] = (row[1] - pad1) / gain;
        row[3] = (row[3] - pad1) / gain;
    }
}

bool YoloKeypointModel::resolve_output_layout(const float *output, int &num_features,
                                              int &num_predictions,
                                              std::vector<float> &transposed_buf,
                                              const float *&prediction_ptr) {
    const std::vector<int64_t> out_shape = engine_.getOutputShape();
    if (out_shape.size() < 3) {
        diagnostics_logger_->warning({}, "Invalid output dimensions");
        return false;
    }
    const int dim1 = static_cast<int>(out_shape[1]);
    const int dim2 = static_cast<int>(out_shape[2]);
    if (dim1 > dim2) {
        num_predictions = dim1;
        num_features = dim2;
        prediction_ptr = output;
    } else {
        num_features = dim1;
        num_predictions = dim2;
        transposed_buf.resize(static_cast<size_t>(num_predictions) * num_features);
        for (int p = 0; p < num_predictions; p++)
            for (int f = 0; f < num_features; f++)
                transposed_buf[static_cast<size_t>(p) * num_features + f] =
                    output[static_cast<size_t>(f) * num_predictions + p];
        prediction_ptr = transposed_buf.data();
    }
    return true;
}

int YoloKeypointModel::extract_keypoints_from_detections(const std::vector<DetectionRow> &keep,
                                                         const Header &header,
                                                         cv::Size original_image_size,
                                                         cv::Size input_image_size,
                                                         KeypointsStamped &result) {
    const int num_classes = static_cast<int>(label_map_.size());
    const int keypoint_start_idx = 6;
    int valid_detections = 0;

    for (int i = 0; i < static_cast<int>(keep.size()); i++) {
        const DetectionRow &row = keep[i];
        if (row.size() < 6) continue;
        const float confidence = row[4];
        if (confidence < threshold_) continue;
        valid_detections++;

        const int class_id = static_cast<int>(row[5]);
        if (class_id >= num_classes) {
            diagnostics_logger_->warning({}, "Invalid class ID: " + std::to_string(class_id));
            continue;
        }
        const Label object_label = label_indices_[class_id];

        DiagnosticsData diag_data;
        diag_data["confidence"] = confidence;
        diag_data["class_id"] = class_id;
        diag_data["stamp"] = header.stamp;
        diagnostics_logger_->info(
            std::string(magic_enum::enum_name(object_label)) + "-" + std::to_string(i), diag_data);

        const std::vector<KeypointLabel> &keypoint_labels =
            label_map_.get_keypoint_labels(object_label);
        const int n_kp = static_cast<int>(keypoint_labels.size());

        for (int k = 0; k < n_kp; k++) {
            const int kp_idx = keypoint_start_idx + k * 3;
            if (kp_idx + 2 >= static_cast<int>(row.size())) break;
            const float kp_conf = row[static_cast<size_t>(kp_idx + 2)];
            if (kp_conf < threshold_) continue;
            Keypoint keypoint;
            keypoint.x = row[static_cast<size_t>(kp_idx)];
            keypoint.y = row[static_cast<size_t>(kp_idx + 1)];
            keypoint = scale_keypoint(keypoint, original_image_size, input_image_size);
            keypoint.keypoint_label = keypoint_labels[static_cast<size_t>(k)];
            keypoint.label = object_label;
            keypoint.confidence = static_cast<double>(confidence);
            keypoint.detection_index = i;
            result.keypoints.push_back(keypoint);
        }
    }
    return valid_detections;
}

KeypointsStamped YoloKeypointModel::postprocess_output(const float *output, const Header &header,
                                                       cv::Size original_image_size,
                                                       cv::Size input_image_size,
                                                       const cv::Mat &original_image) {
    KeypointsStamped result;
    result.header = header;

    int num_features = 0;
    int num_predictions = 0;
    std::vector<float> transposed_buf;
    const float *prediction_ptr = nullptr;
    if (!resolve_output_layout(output, num_features, num_predictions, transposed_buf,
                               prediction_ptr)) {
        return result;
    }

    int num_keypoints = 0;
    if (!label_map_.label_to_keypoints.empty()) {
        num_keypoints = static_cast<int>(label_map_.label_to_keypoints.front().second.size());
    }

    std::vector<DetectionRow> keep =
        non_max_suppression(prediction_ptr, num_features, num_predictions, num_keypoints,
                            threshold_, iou_threshold_, 300);

    const int num_detections = static_cast<int>(keep.size());

    if (num_detections == 0) {
        DiagnosticsData summary_data;
        summary_data["total_detections"] = 0;
        summary_data["valid_detections"] = 0;
        summary_data["threshold"] = threshold_;
        summary_data["iou_threshold"] = iou_threshold_;
        diagnostics_logger_->info(summary_data);
        return result;
    }

    const int valid_detections = extract_keypoints_from_detections(
        keep, header, original_image_size, input_image_size, result);

    DiagnosticsData summary_data;
    summary_data["total_detections"] = num_detections;
    summary_data["valid_detections"] = valid_detections;
    summary_data["threshold"] = threshold_;
    summary_data["iou_threshold"] = iou_threshold_;
    diagnostics_logger_->info(summary_data);

    if (debug_visualization_) {
        scale_boxes(keep, original_image_size, input_image_size);
        visualize_output(original_image, result, keep);
    }

    return result;
}

void YoloKeypointModel::visualize_output(const cv::Mat &original_image,
                                         const KeypointsStamped &keypoints,
                                         const std::vector<DetectionRow> &detections) {
    if (original_image.empty()) return;

    cv::Mat vis_img = original_image.clone();

    std::map<Label, std::vector<cv::Point>> keypoints_by_label;
    for (const auto &kp : keypoints.keypoints) {
        const int x = static_cast<int>(std::round(kp.x));
        const int y = static_cast<int>(std::round(kp.y));
        keypoints_by_label[kp.label].push_back(cv::Point(x, y));
    }

    if (!detections.empty()) {
        struct LabelInfo {
            cv::Rect rect;
            std::string text;
            cv::Scalar color;
            cv::Point text_pos;
        };
        std::vector<LabelInfo> labels;

        for (size_t i = 0; i < detections.size(); i++) {
            const DetectionRow &row = detections[i];
            if (row.size() < 6) continue;
            const float x1 = row[0];
            const float y1 = row[1];
            const float x2 = row[2];
            const float y2 = row[3];
            const float confidence = row[4];
            const int class_id = static_cast<int>(row[5]);
            const Label box_label = label_indices_[class_id];

            auto [b, g, r] = get_color_for_index(box_label).to_bgr_255();
            const cv::Scalar color(b, g, r);

            std::ostringstream label_stream;
            label_stream << std::string(magic_enum::enum_name(box_label)) << " " << std::fixed
                         << std::setprecision(2) << confidence;
            const std::string label_text = label_stream.str();

            cv::rectangle(vis_img, cv::Point(static_cast<int>(x1), static_cast<int>(y1)),
                          cv::Point(static_cast<int>(x2), static_cast<int>(y2)), color, 2);

            int baseline = 0;
            const cv::Size text_size =
                cv::getTextSize(label_text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
            int label_y = static_cast<int>(y1) - text_size.height - 6;
            const int label_x = static_cast<int>(x1);
            cv::Rect proposed_rect(label_x, label_y, text_size.width + 4, text_size.height + 6);
            int offset_count = 0;
            bool has_overlap = true;
            while (has_overlap && offset_count < 10) {
                has_overlap = false;
                for (const auto &existing : labels) {
                    if ((proposed_rect & existing.rect).area() > 0) {
                        has_overlap = true;
                        label_y -= (text_size.height + 8);
                        proposed_rect =
                            cv::Rect(label_x, label_y, text_size.width + 4, text_size.height + 6);
                        offset_count++;
                        break;
                    }
                }
            }
            labels.push_back({proposed_rect, label_text, color,
                              cv::Point(label_x + 2, label_y + text_size.height + 2)});
        }

        for (const auto &label : labels) {
            cv::rectangle(vis_img, label.rect, label.color, cv::FILLED);
            cv::putText(vis_img, label.text, label.text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 255, 255), 2);
        }
    }

    for (const auto &[label, points] : keypoints_by_label) {
        if (points.size() >= 2) {
            auto [b, g, r] = get_color_for_index(label).to_bgr_255();
            cv::Scalar color(b, g, r);
            cv::line(vis_img, points[0], points[1], color, 2, cv::LINE_AA);
        }
    }

    for (const auto &kp : keypoints.keypoints) {
        const int x = static_cast<int>(std::round(kp.x));
        const int y = static_cast<int>(std::round(kp.y));
        auto [b, g, r_val] = get_color_for_index(kp.label).to_bgr_255();
        cv::Scalar color(b, g, r_val);
        const std::string kp_name =
            get_short_name(std::string(magic_enum::enum_name(kp.keypoint_label)));
        cv::circle(vis_img, cv::Point(x, y), 7, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::circle(vis_img, cv::Point(x, y), 5, color, -1, cv::LINE_AA);
        cv::putText(vis_img, kp_name, cv::Point(x + 9, y + 4), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 0), 2);
        cv::putText(vis_img, kp_name, cv::Point(x + 8, y + 3), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1);
    }

    cv::imshow("YOLO Keypoint Visualization", vis_img);
    cv::waitKey(1);
}

}  // namespace auto_battlebot
