/**
 * @file gpu_image_types.cpp
 * @brief GPU to CPU image conversion implementation
 */

#include "communication/gpu_image_types.hpp"

// Only compile CUDA conversion code if CUDA is available
#ifdef CUDA_INTEROP_AVAILABLE
#include <cuda_runtime.h>
#endif

namespace auto_battlebot
{

RgbImage GpuRgbImage::to_cpu([[maybe_unused]] void* stream) const
{
    RgbImage result;
    result.header.frame_id = frame_id;
    result.header.timestamp = timestamp;

    if (!is_valid())
    {
        return result;
    }

#ifdef CUDA_INTEROP_AVAILABLE
    // Allocate CPU image (RGBA format from GPU, will convert to BGR)
    cv::Mat rgba(height, width, CV_8UC4);

    // Copy from cudaArray to host
    // cudaArray is a 2D array, need to use cudaMemcpy2DFromArray
    cudaStream_t cuda_stream = static_cast<cudaStream_t>(stream);

    cudaError_t err = cudaMemcpy2DFromArrayAsync(
        rgba.data,
        rgba.step,                          // dst pitch
        static_cast<cudaArray_t>(cuda_array),
        0, 0,                               // src x, y offset
        width * 4,                          // width in bytes (RGBA)
        height,
        cudaMemcpyDeviceToHost,
        cuda_stream);

    if (err != cudaSuccess)
    {
        // Return empty image on error
        return result;
    }

    // Synchronize if using default stream or wait for async copy
    if (stream)
    {
        cudaStreamSynchronize(cuda_stream);
    }
    else
    {
        cudaDeviceSynchronize();
    }

    // Convert RGBA to BGR (OpenCV format)
    cv::cvtColor(rgba, result.image, cv::COLOR_RGBA2BGR);
#else
    // No CUDA - return placeholder
    result.image = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
#endif

    return result;
}

DepthImage GpuDepthImage::to_cpu([[maybe_unused]] void* stream) const
{
    DepthImage result;
    result.header.frame_id = frame_id;
    result.header.timestamp = timestamp;

    if (!is_valid())
    {
        return result;
    }

#ifdef CUDA_INTEROP_AVAILABLE
    // Allocate CPU image (R32F format)
    result.image = cv::Mat(height, width, CV_32FC1);

    cudaStream_t cuda_stream = static_cast<cudaStream_t>(stream);

    cudaError_t err = cudaMemcpy2DFromArrayAsync(
        result.image.data,
        result.image.step,                  // dst pitch
        static_cast<cudaArray_t>(cuda_array),
        0, 0,                               // src x, y offset
        width * sizeof(float),              // width in bytes
        height,
        cudaMemcpyDeviceToHost,
        cuda_stream);

    if (err != cudaSuccess)
    {
        result.image.release();
        return result;
    }

    // Synchronize
    if (stream)
    {
        cudaStreamSynchronize(cuda_stream);
    }
    else
    {
        cudaDeviceSynchronize();
    }
#else
    // No CUDA - return placeholder
    result.image = cv::Mat(height, width, CV_32FC1, cv::Scalar(0.0f));
#endif

    return result;
}

} // namespace auto_battlebot
