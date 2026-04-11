#include "mask_model/yolo_seg_mask_model.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "diagnostics_logger/function_timer.hpp"

namespace auto_battlebot {
namespace {
constexpr float kMinBoxEdge = 1.0f;

float sigmoid(float value) { return 1.0f / (1.0f + std::exp(-value)); }
}  // namespace

YoloSegMaskModel::YoloSegMaskModel(YoloSegMaskModelConfiguration &config)
    : model_path_(config.model_path),
      confidence_threshold_(config.confidence_threshold),
      iou_threshold_(config.iou_threshold),
      mask_threshold_(config.mask_threshold),
      letterbox_padding_(config.letterbox_padding),
      image_size_(config.image_size),
      max_detections_(config.max_detections),
      debug_visualization_(config.debug_visualization),
      output_label_(config.output_label),
      label_indices_(config.label_indices) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("yolo_seg_mask_model");

    for (size_t i = 0; i < label_indices_.size(); ++i) {
        if (label_indices_[i] == output_label_) {
            target_class_index_ = static_cast<int>(i);
            break;
        }
    }
    if (target_class_index_ < 0) {
        diagnostics_logger_->error({},
                                   "output_label not found in label_indices, mask will stay empty");
    }
    output_mask_value_ = static_cast<uint8_t>(std::max(1, target_class_index_));
}

bool YoloSegMaskModel::initialize() {
    if (target_class_index_ < 0) {
        spdlog::error("YoloSegMaskModel cannot initialize: output_label not found in label_indices");
        return false;
    }

    spdlog::info("Loading YOLO-seg TensorRT engine from: {}", model_path_);
    if (!engine_.load(model_path_)) {
        spdlog::error("Failed to load YOLO-seg TensorRT engine: {}", model_path_);
        return false;
    }

    std::vector<float> warmup_input(static_cast<size_t>(engine_.getInputNumElements()), 0.0f);
    std::vector<float> warmup_output(static_cast<size_t>(engine_.getOutputNumElements()), 0.0f);
    for (int i = 0; i < 2; ++i) {
        if (!engine_.execute(warmup_input.data(), warmup_output.data())) {
            spdlog::error("YOLO-seg warmup inference failed");
            return false;
        }
    }

    initialized_ = true;
    diagnostics_logger_->info({}, "YoloSegMaskModel initialized successfully");
    return true;
}

MaskStamped YoloSegMaskModel::update(RgbImage image) {
    FunctionTimer timer(diagnostics_logger_, "update", 1000.0);
    MaskStamped result;
    result.header = image.header;
    result.mask.label = output_label_;

    if (!initialized_) {
        diagnostics_logger_->error({}, "Model not initialized");
        return result;
    }
    if (image.image.empty()) {
        diagnostics_logger_->error({}, "Input image is empty");
        return result;
    }

    const std::vector<int64_t> in_shape = engine_.getInputShape();
    if (in_shape.size() < 4 || in_shape[0] != 1 || in_shape[1] != 3) {
        diagnostics_logger_->error({}, "YOLO-seg input shape invalid");
        return result;
    }

    const cv::Size input_size(static_cast<int>(in_shape[3]), static_cast<int>(in_shape[2]));
    const cv::Size original_size(image.image.cols, image.image.rows);
    cv::Mat merged_mask = cv::Mat::zeros(original_size, CV_8UC1);

    std::vector<float> input_buffer;
    preprocess_image(image.image, input_size, input_buffer);
    if (static_cast<int64_t>(input_buffer.size()) != engine_.getInputNumElements()) {
        diagnostics_logger_->error({}, "YOLO-seg input buffer size mismatch");
        return result;
    }

    std::vector<TrtEngine::OutputTensorInfo> output_infos = engine_.getOutputTensorInfos();
    if (output_infos.empty()) {
        diagnostics_logger_->error({}, "YOLO-seg output metadata unavailable");
        return result;
    }

    std::vector<std::vector<float>> output_buffers;
    output_buffers.reserve(output_infos.size());
    for (const auto &info : output_infos) {
        output_buffers.emplace_back(static_cast<size_t>(info.num_elements), 0.0f);
    }

    std::vector<float *> output_ptrs;
    output_ptrs.reserve(output_buffers.size());
    for (auto &buffer : output_buffers) output_ptrs.push_back(buffer.data());

    if (!engine_.execute_multi(input_buffer.data(), output_ptrs)) {
        diagnostics_logger_->error({}, "YOLO-seg inference failed");
        return result;
    }

    size_t det_idx = 0;
    size_t proto_idx = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < output_infos.size(); ++i) {
        if (output_infos[i].shape.size() == 4) {
            proto_idx = i;
        }
        if (output_infos[i].num_elements > output_infos[det_idx].num_elements) {
            det_idx = i;
        }
    }

    int proto_channels = 0;
    if (proto_idx != std::numeric_limits<size_t>::max() && output_infos[proto_idx].shape.size() == 4) {
        proto_channels = static_cast<int>(output_infos[proto_idx].shape[1]);
    }

    auto detections =
        decode_detections(output_buffers[det_idx], output_infos[det_idx].shape, proto_channels);
    detections.erase(
        std::remove_if(detections.begin(), detections.end(),
                       [&](const Detection &det) { return det.class_id != target_class_index_; }),
        detections.end());
    detections = non_max_suppression(detections);
    if (static_cast<int>(detections.size()) > max_detections_) {
        detections.resize(static_cast<size_t>(max_detections_));
    }

    for (const auto &det : detections) {
        if (proto_idx == std::numeric_limits<size_t>::max() || det.mask_coeffs.empty()) {
            continue;
        }
        cv::Mat instance_mask =
            decode_instance_mask(det, output_buffers[proto_idx], output_infos[proto_idx].shape, input_size);
        cv::Mat mapped_mask = map_mask_to_original_image(instance_mask, original_size, input_size);
        if (mapped_mask.empty()) continue;
        merged_mask.setTo(output_mask_value_, mapped_mask > 0);
    }

    if (debug_visualization_ && !merged_mask.empty()) {
        diagnostics_logger_->debug(
            {{"nonzero_pixels", std::to_string(cv::countNonZero(merged_mask))},
             {"num_detections", std::to_string(detections.size())}});
    }

    result.mask.mask = merged_mask;
    return result;
}

void YoloSegMaskModel::preprocess_image(const cv::Mat &image, cv::Size input_size,
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

float YoloSegMaskModel::generate_scale(cv::Mat &image, const std::vector<int> &target_size) {
    const float ratio_h = static_cast<float>(target_size[0]) / static_cast<float>(image.rows);
    const float ratio_w = static_cast<float>(target_size[1]) / static_cast<float>(image.cols);
    return std::min(ratio_h, ratio_w);
}

float YoloSegMaskModel::letterbox(cv::Mat &input_image, cv::Mat &output_image,
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

std::vector<YoloSegMaskModel::Detection> YoloSegMaskModel::decode_detections(
    const std::vector<float> &det_output, const std::vector<int64_t> &det_shape,
    int proto_channels) const {
    std::vector<Detection> detections;
    if (det_shape.size() < 3) return detections;

    const int feature_dim = static_cast<int>(det_shape[1]);
    const int num_predictions = static_cast<int>(det_shape[2]);
    const int class_dim = static_cast<int>(label_indices_.size());
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

        const int coeff_start = 4 + class_dim;
        const int coeff_count = std::min(proto_channels, feature_dim - coeff_start);
        det.mask_coeffs.reserve(static_cast<size_t>(coeff_count));
        for (int k = 0; k < coeff_count; ++k) {
            det.mask_coeffs.push_back(read_feature(i, coeff_start + k));
        }
        detections.push_back(det);
    }
    return detections;
}

std::vector<YoloSegMaskModel::Detection> YoloSegMaskModel::non_max_suppression(
    const std::vector<Detection> &detections) const {
    std::vector<Detection> ordered = detections;
    std::sort(ordered.begin(), ordered.end(),
              [](const Detection &lhs, const Detection &rhs) { return lhs.confidence > rhs.confidence; });

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

float YoloSegMaskModel::iou(const Detection &lhs, const Detection &rhs) {
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

cv::Mat YoloSegMaskModel::decode_instance_mask(const Detection &det,
                                               const std::vector<float> &proto_output,
                                               const std::vector<int64_t> &proto_shape,
                                               cv::Size input_size) const {
    if (proto_shape.size() < 4) return cv::Mat{};
    const int channels = static_cast<int>(proto_shape[1]);
    const int proto_h = static_cast<int>(proto_shape[2]);
    const int proto_w = static_cast<int>(proto_shape[3]);
    if (channels <= 0 || proto_h <= 0 || proto_w <= 0 || det.mask_coeffs.empty()) {
        return cv::Mat{};
    }

    cv::Mat logits(proto_h, proto_w, CV_32F, cv::Scalar(0.0f));
    const int coeff_count = std::min(channels, static_cast<int>(det.mask_coeffs.size()));
    for (int c = 0; c < coeff_count; ++c) {
        const float coeff = det.mask_coeffs[static_cast<size_t>(c)];
        const size_t channel_offset = static_cast<size_t>(c) * proto_h * proto_w;
        for (int y = 0; y < proto_h; ++y) {
            float *row = logits.ptr<float>(y);
            const size_t row_offset = channel_offset + static_cast<size_t>(y) * proto_w;
            for (int x = 0; x < proto_w; ++x) {
                row[x] += coeff * proto_output[row_offset + x];
            }
        }
    }

    cv::Mat probs(proto_h, proto_w, CV_32F);
    for (int y = 0; y < proto_h; ++y) {
        const float *in_row = logits.ptr<float>(y);
        float *out_row = probs.ptr<float>(y);
        for (int x = 0; x < proto_w; ++x) {
            out_row[x] = sigmoid(in_row[x]);
        }
    }

    cv::Mat resized;
    cv::resize(probs, resized, input_size, 0, 0, cv::INTER_LINEAR);

    cv::Mat binary = cv::Mat::zeros(input_size, CV_8UC1);
    for (int y = 0; y < resized.rows; ++y) {
        const float *in_row = resized.ptr<float>(y);
        uint8_t *out_row = binary.ptr<uint8_t>(y);
        for (int x = 0; x < resized.cols; ++x) {
            out_row[x] = (in_row[x] >= mask_threshold_) ? 255 : 0;
        }
    }

    cv::Rect bbox(static_cast<int>(std::floor(det.x1)), static_cast<int>(std::floor(det.y1)),
                  static_cast<int>(std::ceil(det.x2 - det.x1)),
                  static_cast<int>(std::ceil(det.y2 - det.y1)));
    bbox &= cv::Rect(0, 0, input_size.width, input_size.height);
    if (bbox.area() > 0) {
        cv::Mat cropped = cv::Mat::zeros(binary.size(), binary.type());
        binary(bbox).copyTo(cropped(bbox));
        return cropped;
    }
    return binary;
}

cv::Mat YoloSegMaskModel::map_mask_to_original_image(const cv::Mat &input_mask, cv::Size original_size,
                                                      cv::Size input_size) const {
    if (input_mask.empty()) return input_mask;
    const double gain = std::min(static_cast<double>(input_size.width) / original_size.width,
                                 static_cast<double>(input_size.height) / original_size.height);
    const double pad_w =
        std::round((input_size.width - original_size.width * gain) / 2.0 - letterbox_padding_);
    const double pad_h =
        std::round((input_size.height - original_size.height * gain) / 2.0 - letterbox_padding_);

    const int x = std::max(0, static_cast<int>(pad_w));
    const int y = std::max(0, static_cast<int>(pad_h));
    const int w = std::max(1, std::min(input_size.width - x,
                                       static_cast<int>(std::round(original_size.width * gain))));
    const int h = std::max(1, std::min(input_size.height - y,
                                       static_cast<int>(std::round(original_size.height * gain))));
    cv::Rect roi(x, y, w, h);
    cv::Mat unpadded = input_mask(roi).clone();
    cv::Mat original;
    cv::resize(unpadded, original, original_size, 0, 0, cv::INTER_NEAREST);
    return original;
}

}  // namespace auto_battlebot
