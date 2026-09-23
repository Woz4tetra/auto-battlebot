#pragma once

#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <string_view>
#include <vector>

namespace auto_battlebot {

/** BGR frame in, JPEG bytes out. One instance per thread. */
class JpegEncoderInterface {
   public:
    virtual ~JpegEncoderInterface() = default;
    /** Called only from the owning thread. Returns false and leaves `out` empty on failure. */
    virtual bool encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) = 0;
    virtual std::string_view name() const = 0;
};

/** cv::imencode on the CPU. The fallback on the desktop and wherever NVJPG fails to open. */
class OpenCvJpegEncoder : public JpegEncoderInterface {
   public:
    bool encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) override;
    std::string_view name() const override { return "opencv"; }

   private:
    std::vector<uint8_t> buffer_;
};

/** The Jetson's NVJPG engine when this build has it and it opens, then OpenCV. Logs which one
 *  it picked. No config field: where NVJPG exists it is always the right choice. */
std::unique_ptr<JpegEncoderInterface> make_jpeg_encoder();

}  // namespace auto_battlebot
