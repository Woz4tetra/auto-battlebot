// Built as its own library, never into auto_battlebot_lib: see NvjpgJpegEncoder.

#include "viz/nvjpg_plugin.h"

#include <NvBuffer.h>
#include <NvJpegEncoder.h>
#include <linux/videodev2.h>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

#define AUTO_BATTLEBOT_NVJPG_EXPORT __attribute__((visibility("default")))

namespace {

struct Encoder {
    NvJPEGEncoder *jpeg = nullptr;
    std::unique_ptr<NvBuffer> buffer;
    int width = 0;
    int height = 0;
    std::vector<unsigned char> out;

    ~Encoder() {
        buffer.reset();
        delete jpeg;
    }

    bool ensure_buffer(int w, int h) {
        if (buffer && w == width && h == height) return true;
        buffer = std::make_unique<NvBuffer>(V4L2_PIX_FMT_YUV420M, static_cast<uint32_t>(w),
                                            static_cast<uint32_t>(h), 0);
        if (buffer->allocateMemory() != 0) {
            buffer.reset();
            return false;
        }
        width = w;
        height = h;
        // Worst case for a JPEG is well under the raw frame size; libjpeg grows it if not.
        out.resize(static_cast<size_t>(w) * static_cast<size_t>(h) * 3 / 2);
        return true;
    }
};

}  // namespace

extern "C" {

AUTO_BATTLEBOT_NVJPG_EXPORT void *auto_battlebot_nvjpg_create(void) {
    auto encoder = std::make_unique<Encoder>();
    encoder->jpeg = NvJPEGEncoder::createJPEGEncoder("auto_battlebot_jpeg");
    if (!encoder->jpeg) return nullptr;
    return encoder.release();
}

AUTO_BATTLEBOT_NVJPG_EXPORT void auto_battlebot_nvjpg_destroy(void *encoder) {
    delete static_cast<Encoder *>(encoder);
}

AUTO_BATTLEBOT_NVJPG_EXPORT size_t auto_battlebot_nvjpg_encode(void *handle,
                                                               const unsigned char *i420, int width,
                                                               int height, int quality,
                                                               const unsigned char **jpeg) {
    auto *encoder = static_cast<Encoder *>(handle);
    if (!encoder || !i420 || width <= 0 || height <= 0 || (width | height) & 1) return 0;
    if (!encoder->ensure_buffer(width, height)) return 0;

    // I420 is one contiguous Y, U, V; NvBuffer has one plane each with its own stride.
    const unsigned char *src = i420;
    for (uint32_t p = 0; p < encoder->buffer->n_planes; ++p) {
        NvBuffer::NvBufferPlane &plane = encoder->buffer->planes[p];
        const uint32_t row_bytes = plane.fmt.width * plane.fmt.bytesperpixel;
        for (uint32_t row = 0; row < plane.fmt.height; ++row) {
            std::memcpy(plane.data + row * plane.fmt.stride, src, row_bytes);
            src += row_bytes;
        }
        plane.bytesused = plane.fmt.stride * plane.fmt.height;
    }

    unsigned char *data = encoder->out.data();
    unsigned long size = encoder->out.size();
    if (encoder->jpeg->encodeFromBuffer(*encoder->buffer, JCS_YCbCr, &data, size, quality) != 0) {
        return 0;
    }
    // libjpeg reallocates when the output outgrows the buffer; keep the larger one.
    if (data != encoder->out.data()) {
        encoder->out.assign(data, data + size);
        std::free(data);
    }
    *jpeg = encoder->out.data();
    return size;
}

}  // extern "C"
