/**
 * @file gpu_image_types.hpp
 * @brief GPU-resident image types for CUDA Interop
 *
 * These types are used only in simulation mode for zero-copy GPU texture
 * sharing via CUDA Interop. They are separate from the core RgbImage and
 * DepthImage types to avoid adding CUDA dependencies to the rest of the stack.
 */

#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>

#include "data_structures/image.hpp"
#include "data_structures/header.hpp"

namespace auto_battlebot
{

/**
 * @brief GPU-resident RGB image from CUDA Interop
 *
 * This struct holds a reference to a cudaArray_t that contains RGB image
 * data directly on the GPU. The data can be used for TensorRT inference
 * without any CPU-GPU transfers.
 */
struct GpuRgbImage
{
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    void* cuda_array = nullptr;  // cudaArray_t (RGBA8 format)
    int width = 0;
    int height = 0;

    /// Check if image data is valid
    bool is_valid() const
    {
        return cuda_array != nullptr && width > 0 && height > 0;
    }

    /**
     * @brief Convert to CPU RgbImage
     *
     * This requires copying data from GPU to CPU. The caller must ensure
     * CUDA resources are still mapped when calling this function.
     *
     * @param stream CUDA stream for async copy (nullptr for default stream)
     * @return RgbImage with data copied to CPU
     *
     * @note This is an expensive operation and should be avoided in the
     *       hot path. Use GPU-resident data directly when possible.
     */
    RgbImage to_cpu(void* stream = nullptr) const;

    /**
     * @brief Create a placeholder CPU image (no actual data copy)
     *
     * Creates an RgbImage with correct header/dimensions but empty cv::Mat.
     * Useful when GPU data will be used directly and CPU copy isn't needed.
     */
    RgbImage to_placeholder() const
    {
        RgbImage result;
        result.header.frame_id = frame_id;
        result.header.timestamp = timestamp;
        // image Mat left empty intentionally
        return result;
    }
};

/**
 * @brief GPU-resident depth image from CUDA Interop
 *
 * This struct holds a reference to a cudaArray_t that contains depth
 * data directly on the GPU (R32F format, values in meters).
 */
struct GpuDepthImage
{
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    void* cuda_array = nullptr;  // cudaArray_t (R32F format)
    int width = 0;
    int height = 0;

    /// Check if depth data is valid
    bool is_valid() const
    {
        return cuda_array != nullptr && width > 0 && height > 0;
    }

    /**
     * @brief Convert to CPU DepthImage
     *
     * This requires copying data from GPU to CPU. The caller must ensure
     * CUDA resources are still mapped when calling this function.
     *
     * @param stream CUDA stream for async copy (nullptr for default stream)
     * @return DepthImage with data copied to CPU
     *
     * @note This is an expensive operation and should be avoided in the
     *       hot path. Use GPU-resident data directly when possible.
     */
    DepthImage to_cpu(void* stream = nullptr) const;

    /**
     * @brief Create a placeholder CPU image (no actual data copy)
     *
     * Creates a DepthImage with correct header/dimensions but empty cv::Mat.
     * Useful when GPU data will be used directly and CPU copy isn't needed.
     */
    DepthImage to_placeholder() const
    {
        DepthImage result;
        result.header.frame_id = frame_id;
        result.header.timestamp = timestamp;
        // image Mat left empty intentionally
        return result;
    }
};

/**
 * @brief Combined GPU frame data from simulation
 *
 * Contains both RGB and depth GPU images along with metadata.
 * Used as output from SimRgbdCamera when CUDA Interop is enabled.
 */
struct GpuFrameData
{
    GpuRgbImage rgb;
    GpuDepthImage depth;

    /// Check if RGB is valid
    bool has_rgb() const { return rgb.is_valid(); }

    /// Check if depth is valid
    bool has_depth() const { return depth.is_valid(); }

    /// Check if frame has any valid data
    bool is_valid() const { return has_rgb() || has_depth(); }
};

} // namespace auto_battlebot
