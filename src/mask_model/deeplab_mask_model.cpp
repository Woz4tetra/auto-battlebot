#include "mask_model/deeplab_mask_model.hpp"

#include <spdlog/spdlog.h>
#include <toml++/toml.h>

#include <filesystem>

namespace auto_battlebot {
namespace {
constexpr double kUpdateWarnMs = 1000.0;
}  // namespace
DeepLabMaskModel::DeepLabMaskModel(DeepLabMaskModelConfiguration& config,
                                   std::shared_ptr<EngineSelector> engine_selector)
    : model_type_(config.model_type),
      output_label_(config.output_label),
      engine_selector_(std::move(engine_selector)),
      initialized_(false) {
    diagnostics_logger_ = DiagnosticsLogger::get_logger("deeplab_mask_model");
}

bool DeepLabMaskModel::load_model_config() {
    std::filesystem::path engine_path(model_path_);
    std::filesystem::path toml_path = engine_path;
    toml_path.replace_extension(".toml");

    if (!std::filesystem::exists(toml_path)) {
        spdlog::error("Model config not found: {}", toml_path.string());
        return false;
    }

    try {
        auto config = toml::parse_file(toml_path.string());
        auto model_section = config["model"];
        image_size_ = model_section["image_size"].value_or(0);
        border_padding_ = model_section["pad_size"].value_or(0);

        if (image_size_ <= 0) {
            spdlog::error("Invalid image_size in {}", toml_path.string());
            return false;
        }

        spdlog::info("Model config: image_size={}, border_padding={}", image_size_,
                     border_padding_);
    } catch (const toml::parse_error& e) {
        spdlog::error("Failed to parse model config {}: {}", toml_path.string(), e.description());
        return false;
    }
    return true;
}

bool DeepLabMaskModel::initialize() {
    const std::optional<std::string> selected = engine_selector_->select(engine_);
    if (!selected) {
        return false;
    }
    model_path_ = *selected;

    if (!load_model_config()) {
        return false;
    }

    spdlog::info("Warming up model with dummy input...");
    std::vector<float> warmup_input(static_cast<size_t>(engine_.getInputNumElements()), 0.0f);
    std::vector<float> warmup_output(static_cast<size_t>(engine_.getOutputNumElements()), 0.0f);
    if (!engine_.execute(warmup_input, warmup_output)) {
        spdlog::error("DeepLab warmup inference failed");
        return false;
    }
    spdlog::info("DeepLabMaskModel initialized!");

    initialized_ = true;
    diagnostics_logger_->info({}, "DeepLabMaskModel initialized successfully");
    return true;
}

MaskStamped DeepLabMaskModel::update(RgbImage image) {
    FunctionTimer timer(diagnostics_logger_, "update", kUpdateWarnMs);

    if (!initialized_) {
        diagnostics_logger_->error({}, "Model not initialized");
        return MaskStamped{};
    }

    const int original_height = image.image.rows;
    const int original_width = image.image.cols;

    std::vector<float> input_buffer;
    preprocess_image_to_nchw_float(image.image, input_buffer);

    std::vector<float> output_buffer(static_cast<size_t>(engine_.getOutputNumElements()));

    if (!engine_.execute(input_buffer, output_buffer)) {
        diagnostics_logger_->error({}, "DeepLab inference failed");
        return MaskStamped{};
    }

    cv::Mat mask = postprocess_host_output(output_buffer.data(), original_height, original_width);

    MaskStamped result;
    result.header = image.header;
    result.mask.label = output_label_;
    result.mask.mask = mask;

    return result;
}

void deeplab_input_nchw(const cv::Mat& bgr_image, int model_height, int model_width,
                        int border_padding, std::vector<float>& buffer) {
    cv::Mat rgb_image;
    if (bgr_image.channels() == 4) {
        cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGRA2RGB);
    } else if (bgr_image.channels() == 1) {
        cv::cvtColor(bgr_image, rgb_image, cv::COLOR_GRAY2RGB);
    } else {
        cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
    }

    cv::Mat padded_image;
    if (border_padding > 0) {
        cv::copyMakeBorder(rgb_image, padded_image, border_padding, border_padding, border_padding,
                           border_padding, cv::BORDER_REPLICATE);
    } else {
        padded_image = rgb_image;
    }

    cv::Mat resized;
    cv::resize(padded_image, resized, cv::Size(model_width, model_height));
    cv::Mat float_image;
    resized.convertTo(float_image, CV_32F, 1.0 / 255.0);

    const size_t plane = static_cast<size_t>(model_height) * static_cast<size_t>(model_width);
    buffer.resize(3 * plane);
    for (int y = 0; y < model_height; y++) {
        const cv::Vec3f* row = float_image.ptr<cv::Vec3f>(y);
        for (int x = 0; x < model_width; x++) {
            const size_t offset = static_cast<size_t>(y) * model_width + x;
            for (int c = 0; c < 3; c++) {
                buffer[c * plane + offset] =
                    (row[x][c] - kDeepLabTrainingMeanRgb[c]) / kDeepLabTrainingStdRgb[c];
            }
        }
    }
}

void DeepLabMaskModel::preprocess_image_to_nchw_float(const cv::Mat& image,
                                                      std::vector<float>& buffer) {
    const std::vector<int64_t> in_shape = engine_.getInputShape();
    if (in_shape.size() < 4 || in_shape[0] != 1 || in_shape[1] != 3) {
        return;
    }
    deeplab_input_nchw(image, static_cast<int>(in_shape[2]), static_cast<int>(in_shape[3]),
                       border_padding_, buffer);
}

cv::Mat DeepLabMaskModel::postprocess_host_output(const float* output, int original_height,
                                                  int original_width) {
    const std::vector<int64_t> out_shape = engine_.getOutputShape();
    if (out_shape.size() < 4) {
        return cv::Mat();
    }
    const int64_t num_classes = out_shape[1];
    const int H = static_cast<int>(out_shape[2]);
    const int W = static_cast<int>(out_shape[3]);

    cv::Mat mask(H, W, CV_8UC1);
    uint8_t* mask_ptr = mask.data;

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
