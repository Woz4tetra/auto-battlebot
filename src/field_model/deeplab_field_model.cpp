#include "field_model/deeplab_field_model.hpp"

#include <filesystem>
#include <toml++/toml.h>

namespace auto_battlebot {
DeepLabFieldModel::DeepLabFieldModel(DeepLabFieldModelConfiguration &config)
    : model_path_(config.model_path),
      model_type_(config.model_type),
      initialized_(false) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("deeplab_field_model");
}

bool DeepLabFieldModel::load_model_config() {
    std::filesystem::path engine_path(model_path_);
    std::filesystem::path toml_path = engine_path;
    toml_path.replace_extension(".toml");

    if (!std::filesystem::exists(toml_path)) {
        std::cerr << "Model config not found: " << toml_path << std::endl;
        return false;
    }

    try {
        auto config = toml::parse_file(toml_path.string());
        auto model_section = config["model"];
        image_size_ = model_section["image_size"].value_or(0);
        border_padding_ = model_section["pad_size"].value_or(0);

        if (image_size_ <= 0) {
            std::cerr << "Invalid image_size in " << toml_path << std::endl;
            return false;
        }

        std::cout << "Model config: image_size=" << image_size_
                  << ", border_padding=" << border_padding_ << std::endl;
    } catch (const toml::parse_error &e) {
        std::cerr << "Failed to parse model config " << toml_path << ": " << e.description() << std::endl;
        return false;
    }
    return true;
}

bool DeepLabFieldModel::initialize() {
    if (!load_model_config()) {
        return false;
    }

    std::cout << "Loading TensorRT engine from: " << model_path_ << std::endl;
    if (!engine_.load(model_path_)) {
        std::cerr << "Failed to load DeepLab TensorRT engine: " << model_path_ << std::endl;
        return false;
    }

    // Warmup inference
    std::cout << "Warming up model with dummy input..." << std::endl;
    std::vector<float> warmup_input(static_cast<size_t>(engine_.getInputNumElements()), 0.0f);
    std::vector<float> warmup_output(static_cast<size_t>(engine_.getOutputNumElements()), 0.0f);
    if (!engine_.execute(warmup_input.data(), warmup_output.data())) {
        std::cerr << "DeepLab warmup inference failed" << std::endl;
        return false;
    }
    std::cout << "DeepLabFieldModel initialized!" << std::endl;

    initialized_ = true;
    diagnostics_logger_->info({}, "DeepLabFieldModel initialized successfully");
    return true;
}

FieldMaskStamped DeepLabFieldModel::update(RgbImage image) {
    FunctionTimer timer(diagnostics_logger_, "update", 1000.0);  // Warn if > 1000ms

    if (!initialized_) {
        diagnostics_logger_->error({}, "Model not initialized");
        return FieldMaskStamped{};
    }

    const int original_height = image.image.rows;
    const int original_width = image.image.cols;

    std::vector<float> input_buffer;
    preprocess_image(image.image, input_buffer);

    std::vector<float> output_buffer(static_cast<size_t>(engine_.getOutputNumElements()));

    if (!engine_.execute(input_buffer.data(), output_buffer.data())) {
        diagnostics_logger_->error({}, "DeepLab inference failed");
        return FieldMaskStamped{};
    }

    cv::Mat mask = postprocess_output(output_buffer.data(), original_height, original_width);

    FieldMaskStamped result;
    result.header = image.header;
    result.mask.label = Label::FIELD;
    result.mask.mask = mask;

    return result;
}

void DeepLabFieldModel::preprocess_image(const cv::Mat &image, std::vector<float> &buffer) {
    const std::vector<int64_t> in_shape = engine_.getInputShape();
    if (in_shape.size() < 4 || in_shape[0] != 1 || in_shape[1] != 3) {
        return;
    }
    const int model_h = static_cast<int>(in_shape[2]);
    const int model_w = static_cast<int>(in_shape[3]);

    cv::Mat rgb_image;
    if (image.channels() == 3) {
        rgb_image = image;
    } else if (image.channels() == 4) {
        cv::cvtColor(image, rgb_image, cv::COLOR_BGRA2RGB);
    } else {
        cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
    }

    cv::Mat padded_image;
    if (border_padding_ > 0) {
        cv::copyMakeBorder(rgb_image, padded_image, border_padding_, border_padding_,
                           border_padding_, border_padding_, cv::BORDER_REPLICATE);
    } else {
        padded_image = rgb_image;
    }

    cv::Mat resized;
    cv::resize(padded_image, resized, cv::Size(model_w, model_h));

    cv::Mat float_image;
    resized.convertTo(float_image, CV_32F, 1.0 / 255.0);

    // ImageNet per-channel normalization — must match transforms.Normalize in training pipeline
    const std::vector<float> mean = {0.485f, 0.456f, 0.406f};
    const std::vector<float> std_dev = {0.229f, 0.224f, 0.225f};

    std::vector<cv::Mat> channels(3);
    cv::split(float_image, channels);
    for (int i = 0; i < 3; i++) {
        channels[i] = (channels[i] - mean[i]) / std_dev[i];
    }
    cv::merge(channels, float_image);

    // NCHW: 1 * 3 * H * W — size must match engine input
    const int64_t num_elements = engine_.getInputNumElements();
    buffer.resize(static_cast<size_t>(num_elements));
    float *ptr = buffer.data();
    for (int c = 0; c < 3; c++) {
        for (int y = 0; y < model_h; y++) {
            for (int x = 0; x < model_w; x++) {
                ptr[c * model_h * model_w + y * model_w + x] = float_image.at<cv::Vec3f>(y, x)[c];
            }
        }
    }
}

cv::Mat DeepLabFieldModel::postprocess_output(const float *output, int original_height,
                                              int original_width) {
    const std::vector<int64_t> out_shape = engine_.getOutputShape();
    if (out_shape.size() < 4) {
        return cv::Mat();
    }
    const int64_t num_classes = out_shape[1];
    const int H = static_cast<int>(out_shape[2]);
    const int W = static_cast<int>(out_shape[3]);

    cv::Mat mask(H, W, CV_8UC1);
    uint8_t *mask_ptr = mask.data;

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int best_c = 0;
            float best_val = output[0 * H * W + y * W + x];
            for (int64_t c = 1; c < num_classes; c++) {
                const float v = output[c * H * W + y * W + x];
                if (v > best_val) {
                    best_val = v;
                    best_c = static_cast<int>(c);
                }
            }
            mask_ptr[y * W + x] = static_cast<uint8_t>(best_c);
        }
    }

    const int padded_width = original_width + 2 * border_padding_;
    const int padded_height = original_height + 2 * border_padding_;
    cv::Mat resized_mask;
    cv::resize(mask, resized_mask, cv::Size(padded_width, padded_height), 0, 0, cv::INTER_NEAREST);

    if (border_padding_ > 0) {
        const cv::Rect roi(border_padding_, border_padding_, original_width, original_height);
        resized_mask = resized_mask(roi).clone();
    }

    return resized_mask;
}

}  // namespace auto_battlebot
