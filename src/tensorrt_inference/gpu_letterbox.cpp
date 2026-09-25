#include "tensorrt_inference/gpu_letterbox.hpp"

#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include "tensorrt_inference/letterbox_kernel.hpp"

namespace auto_battlebot {
namespace {

// The device address for `pointer` if the GPU can read it where it is: device or managed memory,
// or pinned host memory mapped into the device's address space. nullptr for pageable memory.
const uint8_t *device_readable(const void *pointer) {
    cudaPointerAttributes attributes{};
    if (cudaPointerGetAttributes(&attributes, pointer) != cudaSuccess) {
        cudaGetLastError();
        return nullptr;
    }
    switch (attributes.type) {
        case cudaMemoryTypeDevice:
        case cudaMemoryTypeManaged:
            return static_cast<const uint8_t *>(pointer);
        case cudaMemoryTypeHost:
            return static_cast<const uint8_t *>(attributes.devicePointer);
        default:
            return nullptr;
    }
}

}  // namespace

GpuLetterbox::~GpuLetterbox() {
    if (staging_) cudaFree(staging_);
}

bool GpuLetterbox::enqueue(const cv::Mat &image, int tensor_width, int tensor_height, float padding,
                           float *tensor, void *stream) {
    if (image.empty() || image.depth() != CV_8U ||
        (image.channels() != 3 && image.channels() != 4)) {
        spdlog::error("GpuLetterbox: needs an 8-bit BGR or BGRA image, got type {}", image.type());
        return false;
    }
    const LetterboxGeometry geometry =
        compute_letterbox_geometry(image.cols, image.rows, tensor_width, tensor_height, padding);
    if (!geometry.fills_tensor) {
        spdlog::error("GpuLetterbox: {}x{} does not letterbox to exactly {}x{}", image.cols,
                      image.rows, tensor_width, tensor_height);
        return false;
    }

    const uint8_t *source = device_readable(image.data);
    size_t source_step = image.step[0];
    last_was_zero_copy_ = source != nullptr;
    if (!source) {
        const size_t row_bytes = static_cast<size_t>(image.cols) * image.elemSize();
        const size_t bytes = row_bytes * static_cast<size_t>(image.rows);
        if (bytes > staging_bytes_) {
            if (staging_) cudaFree(staging_);
            staging_ = nullptr;
            staging_bytes_ = 0;
            const cudaError_t err = cudaMalloc(&staging_, bytes);
            if (err != cudaSuccess) {
                spdlog::error("GpuLetterbox: cudaMalloc staging failed: {}",
                              cudaGetErrorString(err));
                return false;
            }
            staging_bytes_ = bytes;
        }
        // From pageable memory the copy has finished reading `image` by the time it returns.
        const cudaError_t err =
            cudaMemcpy2DAsync(staging_, row_bytes, image.data, image.step[0], row_bytes,
                              static_cast<size_t>(image.rows), cudaMemcpyHostToDevice,
                              static_cast<cudaStream_t>(stream));
        if (err != cudaSuccess) {
            spdlog::error("GpuLetterbox: frame upload failed: {}", cudaGetErrorString(err));
            return false;
        }
        source = static_cast<const uint8_t *>(staging_);
        source_step = row_bytes;
    }

    if (!launch_letterbox_kernel(source, image.cols, image.rows, source_step, image.channels(),
                                 geometry, tensor, stream)) {
        spdlog::error("GpuLetterbox: kernel launch failed");
        return false;
    }
    return true;
}

}  // namespace auto_battlebot
