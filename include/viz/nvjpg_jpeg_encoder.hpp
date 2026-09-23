#pragma once

#include <memory>
#include <opencv2/core.hpp>

#include "viz/jpeg_encoder.hpp"

class NvBuffer;
class NvJPEGEncoder;

namespace auto_battlebot {

/**
 * JPEG on the Jetson's NVJPG engine through NvJPEGEncoder from the Jetson Multimedia API.
 * Compiled only when CMake finds the Multimedia API headers and L4T's libnvjpeg
 * (AUTO_BATTLEBOT_HAVE_NVJPG). That libnvjpeg is NVIDIA's libjpeg-compatible library with the
 * NVJPG extensions, not the CUDA toolkit's GPU nvJPEG: the GPU belongs to the detectors.
 *
 * NVJPG takes YUV 4:2:0, so each frame is converted from BGR on the CPU first (about 0.3 ms at
 * 640x360). Odd widths and heights lose their last column or row, since I420 needs even ones.
 */
class NvjpgJpegEncoder : public JpegEncoderInterface {
   public:
    /** nullptr when the engine does not open. */
    static std::unique_ptr<NvjpgJpegEncoder> create();
    ~NvjpgJpegEncoder() override;

    bool encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) override;
    std::string_view name() const override { return "nvjpg"; }

   private:
    NvjpgJpegEncoder() = default;
    bool ensure_buffer(int width, int height);

    NvJPEGEncoder *encoder_ = nullptr;
    std::unique_ptr<NvBuffer> buffer_;
    int width_ = 0;
    int height_ = 0;
    cv::Mat i420_;
    std::vector<unsigned char> out_;
};

}  // namespace auto_battlebot
