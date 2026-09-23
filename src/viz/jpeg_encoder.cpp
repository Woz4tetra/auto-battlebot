#include "viz/jpeg_encoder.hpp"

#include <spdlog/spdlog.h>

#include <atomic>
#include <opencv2/imgcodecs.hpp>

#ifdef AUTO_BATTLEBOT_HAVE_NVJPG
#include "viz/nvjpg_jpeg_encoder.hpp"
#endif

namespace auto_battlebot {

bool OpenCvJpegEncoder::encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) {
    out.clear();
    if (bgr.empty()) return false;
    if (!cv::imencode(".jpg", bgr, buffer_, {cv::IMWRITE_JPEG_QUALITY, quality})) return false;
    const auto *begin = reinterpret_cast<const std::byte *>(buffer_.data());
    out.assign(begin, begin + buffer_.size());
    return true;
}

std::unique_ptr<JpegEncoderInterface> make_jpeg_encoder() {
    // Logged once per process, not once per encoder.
    static std::atomic<bool> logged{false};
    std::unique_ptr<JpegEncoderInterface> encoder;
#ifdef AUTO_BATTLEBOT_HAVE_NVJPG
    encoder = NvjpgJpegEncoder::create();
#endif
    if (!encoder) encoder = std::make_unique<OpenCvJpegEncoder>();
    if (!logged.exchange(true)) spdlog::info("JPEG encoder: {}", encoder->name());
    return encoder;
}

}  // namespace auto_battlebot
