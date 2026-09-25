#include <cuda_runtime.h>

#include "tensorrt_inference/letterbox_kernel.hpp"

namespace auto_battlebot {
namespace {

// cv::resize INTER_LINEAR's source position for output index `d`: pixel centres line up, and a
// position past either edge clamps to that edge with no blend.
__device__ inline void source_index(int d, float scale, int size, int &i0, int &i1, float &frac) {
    float f = (static_cast<float>(d) + 0.5f) * scale - 0.5f;
    int i = static_cast<int>(floorf(f));
    f -= static_cast<float>(i);
    if (i < 0) {
        i = 0;
        f = 0.0f;
    }
    if (i >= size - 1) {
        i = size - 1;
        f = 0.0f;
    }
    i0 = i;
    i1 = min(i + 1, size - 1);
    frac = f;
}

// One thread per tensor pixel, writing all three planes.
__global__ void letterbox(const uint8_t *__restrict__ source, int source_width, int source_height,
                          size_t source_step, int source_channels, LetterboxGeometry geometry,
                          float scale_x, float scale_y, float *__restrict__ tensor) {
    const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
    const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
    if (x >= geometry.tensor_width || y >= geometry.tensor_height) return;

    const size_t plane = static_cast<size_t>(geometry.tensor_width) * geometry.tensor_height;
    float *out = tensor + static_cast<size_t>(y) * geometry.tensor_width + x;

    const int rx = x - geometry.left;
    const int ry = y - geometry.top;
    if (rx < 0 || ry < 0 || rx >= geometry.resized_width || ry >= geometry.resized_height) {
        out[0] = kLetterboxPadValue;
        out[plane] = kLetterboxPadValue;
        out[2 * plane] = kLetterboxPadValue;
        return;
    }

    int x0, x1, y0, y1;
    float fx, fy;
    source_index(rx, scale_x, source_width, x0, x1, fx);
    source_index(ry, scale_y, source_height, y0, y1, fy);
    const uint8_t *row0 = source + static_cast<size_t>(y0) * source_step;
    const uint8_t *row1 = source + static_cast<size_t>(y1) * source_step;
    const int c0 = x0 * source_channels;
    const int c1 = x1 * source_channels;

    // Source is BGR(A); the tensor is RGB, so source channel c goes to plane 2 - c.
    for (int c = 0; c < 3; ++c) {
        const float top = (1.0f - fx) * row0[c0 + c] + fx * row0[c1 + c];
        const float bottom = (1.0f - fx) * row1[c0 + c] + fx * row1[c1 + c];
        out[(2 - c) * plane] = ((1.0f - fy) * top + fy * bottom) * (1.0f / 255.0f);
    }
}

}  // namespace

bool launch_letterbox_kernel(const uint8_t *source, int source_width, int source_height,
                             size_t source_step, int source_channels,
                             const LetterboxGeometry &geometry, float *tensor, void *stream) {
    if (geometry.resized_width <= 0 || geometry.resized_height <= 0) return false;
    // cv::resize's inverse scale, computed in double as it does.
    const float scale_x = static_cast<float>(static_cast<double>(source_width) /
                                             static_cast<double>(geometry.resized_width));
    const float scale_y = static_cast<float>(static_cast<double>(source_height) /
                                             static_cast<double>(geometry.resized_height));
    const dim3 block(32, 8);
    const dim3 grid((geometry.tensor_width + block.x - 1) / block.x,
                    (geometry.tensor_height + block.y - 1) / block.y);
    letterbox<<<grid, block, 0, static_cast<cudaStream_t>(stream)>>>(
        source, source_width, source_height, source_step, source_channels, geometry, scale_x,
        scale_y, tensor);
    return cudaGetLastError() == cudaSuccess;
}

}  // namespace auto_battlebot
