#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Shared by the .cu kernel and C++ callers, so this header includes neither OpenCV nor CUDA.

namespace auto_battlebot {

// Where a source image lands inside a YOLO input tensor: scaled to fit, centred, the rest padded.
struct LetterboxGeometry {
    int tensor_width = 0;
    int tensor_height = 0;
    // The scaled image's size and its offset inside the tensor.
    int resized_width = 0;
    int resized_height = 0;
    int left = 0;
    int top = 0;
    // False when the rounded padding does not add up to the tensor size. The CPU letterbox
    // reported that as an input size mismatch.
    bool fills_tensor = true;
};

// The same arithmetic as YOLO's letterbox and as YoloKeypointModel::scale_boxes, which maps
// detections back: fit the long side, then split the padding with `padding` biasing each side.
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

// Grey the YOLO letterbox pads with, as a 0-1 value.
constexpr float kLetterboxPadValue = 114.0f / 255.0f;

// Enqueues one kernel on `stream` (a cudaStream_t) that letterboxes an 8-bit BGR or BGRA image
// into `tensor`: bilinear resize with cv::resize INTER_LINEAR's sample positions, BGR to RGB,
// scaled to 0-1, planar CHW. `source` must be readable from the device and `tensor` must hold
// 3 * tensor_height * tensor_width floats. Returns false if the launch failed.
bool launch_letterbox_kernel(const uint8_t *source, int source_width, int source_height,
                             size_t source_step, int source_channels,
                             const LetterboxGeometry &geometry, float *tensor, void *stream);

}  // namespace auto_battlebot
