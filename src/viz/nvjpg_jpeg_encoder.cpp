#include "viz/nvjpg_jpeg_encoder.hpp"

#include <NvBuffer.h>
#include <NvJpegEncoder.h>
#include <linux/videodev2.h>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <cstring>
#include <opencv2/imgproc.hpp>

namespace auto_battlebot {

std::unique_ptr<NvjpgJpegEncoder> NvjpgJpegEncoder::create() {
    std::unique_ptr<NvjpgJpegEncoder> out(new NvjpgJpegEncoder());
    out->encoder_ = NvJPEGEncoder::createJPEGEncoder("auto_battlebot_jpeg");
    if (!out->encoder_) {
        spdlog::warn("NVJPG encoder did not open; using OpenCV");
        return nullptr;
    }
    return out;
}

NvjpgJpegEncoder::~NvjpgJpegEncoder() {
    buffer_.reset();
    delete encoder_;
}

bool NvjpgJpegEncoder::ensure_buffer(int width, int height) {
    if (buffer_ && width == width_ && height == height_) return true;
    buffer_ = std::make_unique<NvBuffer>(V4L2_PIX_FMT_YUV420M, static_cast<uint32_t>(width),
                                         static_cast<uint32_t>(height), 0);
    if (buffer_->allocateMemory() != 0) {
        spdlog::error("NVJPG: could not allocate a {}x{} YUV420 buffer", width, height);
        buffer_.reset();
        return false;
    }
    width_ = width;
    height_ = height;
    // Worst case for a JPEG is well under the raw frame size; libjpeg grows it if not.
    out_.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3 / 2);
    return true;
}

bool NvjpgJpegEncoder::encode(const cv::Mat &bgr, int quality, std::vector<std::byte> &out) {
    out.clear();
    if (bgr.empty() || bgr.type() != CV_8UC3) return false;
    const int width = bgr.cols & ~1;
    const int height = bgr.rows & ~1;
    if (width == 0 || height == 0 || !ensure_buffer(width, height)) return false;

    cv::cvtColor(bgr(cv::Rect(0, 0, width, height)), i420_, cv::COLOR_BGR2YUV_I420);

    // I420 is one contiguous Y, U, V; NvBuffer has one plane each with its own stride.
    const unsigned char *src = i420_.ptr<unsigned char>();
    for (uint32_t p = 0; p < buffer_->n_planes; ++p) {
        NvBuffer::NvBufferPlane &plane = buffer_->planes[p];
        const uint32_t row_bytes = plane.fmt.width * plane.fmt.bytesperpixel;
        for (uint32_t row = 0; row < plane.fmt.height; ++row) {
            std::memcpy(plane.data + row * plane.fmt.stride, src, row_bytes);
            src += row_bytes;
        }
        plane.bytesused = plane.fmt.stride * plane.fmt.height;
    }

    unsigned char *data = out_.data();
    unsigned long size = out_.size();
    if (encoder_->encodeFromBuffer(*buffer_, JCS_YCbCr, &data, size, quality) != 0) return false;
    const auto *begin = reinterpret_cast<const std::byte *>(data);
    out.assign(begin, begin + size);
    // libjpeg reallocates when the output outgrows the buffer; keep the larger one.
    if (data != out_.data()) {
        out_.assign(data, data + size);
        std::free(data);
    }
    return true;
}

}  // namespace auto_battlebot
