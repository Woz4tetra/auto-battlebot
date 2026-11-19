#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "tensorrt_inference/cuda_handles.hpp"

namespace auto_battlebot {

struct LetterboxGeometry {
    int tensor_width = 0;
    int tensor_height = 0;
    int resized_width = 0;
    int resized_height = 0;
    int left = 0;
    int top = 0;
    bool fills_tensor = true;
};

inline LetterboxGeometry compute_letterbox_geometry(int source_width, int source_height,
                                                    int tensor_width, int tensor_height,
                                                    float padding) {
    LetterboxGeometry geometry;
    geometry.tensor_width = tensor_width;
    geometry.tensor_height = tensor_height;
    if (source_width == tensor_width && source_height == tensor_height) {
        geometry.resized_width = source_width;
        geometry.resized_height = source_height;
        return geometry;
    }
    const float scale =
        std::min(static_cast<float>(tensor_height) / static_cast<float>(source_height),
                 static_cast<float>(tensor_width) / static_cast<float>(source_width));
    geometry.resized_width = static_cast<int>(std::round(static_cast<float>(source_width) * scale));
    geometry.resized_height =
        static_cast<int>(std::round(static_cast<float>(source_height) * scale));
    const float pad_w = static_cast<float>(tensor_width - geometry.resized_width) / 2.0f;
    const float pad_h = static_cast<float>(tensor_height - geometry.resized_height) / 2.0f;
    geometry.left = static_cast<int>(std::round(pad_w - padding));
    geometry.top = static_cast<int>(std::round(pad_h - padding));
    const int right = static_cast<int>(std::round(pad_w + padding));
    const int bottom = static_cast<int>(std::round(pad_h + padding));
    geometry.fills_tensor = geometry.left + geometry.resized_width + right == tensor_width &&
                            geometry.top + geometry.resized_height + bottom == tensor_height;
    return geometry;
}

constexpr float kYoloPadGrey01 = 114.0f / 255.0f;

bool launch_letterbox_kernel(const uint8_t *source, int source_width, int source_height,
                             size_t source_step, int source_channels,
                             const LetterboxGeometry &geometry, float *tensor, cudaStream_t stream);

}  // namespace auto_battlebot
