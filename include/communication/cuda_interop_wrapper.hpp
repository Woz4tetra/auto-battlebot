/**
 * @file cuda_interop_wrapper.hpp
 * @brief C++ wrapper for the CUDA Interop native plugin
 *
 * This class provides a RAII-safe wrapper around the C API defined in
 * cuda_interop.h. It handles initialization, resource mapping/unmapping,
 * and provides access to GPU texture data for inference.
 */

#pragma once

#include <string>
#include <optional>
#include <memory>
#include <mutex>

#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{

/**
 * @brief Texture types for CUDA Interop
 */
enum class CudaInteropTextureType
{
    RGB = 0,
    Depth = 1
};

/**
 * @brief Frame information from CUDA Interop
 */
struct CudaInteropFrameInfo
{
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    int rgb_width = 0;
    int rgb_height = 0;
    int depth_width = 0;
    int depth_height = 0;
    bool rgb_valid = false;
    bool depth_valid = false;
};

/**
 * @brief Performance metrics from CUDA Interop
 */
struct CudaInteropMetrics
{
    double last_map_time_ms = 0.0;
    double last_unmap_time_ms = 0.0;
    double last_sync_time_ms = 0.0;
    double avg_map_time_ms = 0.0;
    double avg_unmap_time_ms = 0.0;
    uint64_t total_frames = 0;
    uint64_t map_errors = 0;
    uint64_t unmap_errors = 0;
};

/**
 * @brief Configuration for CUDA Interop
 */
struct CudaInteropConfig
{
    int cuda_device_id = 0;
    std::string plugin_path;  // Path to native plugin (optional, for dynamic loading)
    bool enable_metrics = true;
};

/**
 * @brief C++ wrapper for CUDA Interop native plugin
 *
 * This class provides a safe, RAII-style interface to the CUDA Interop
 * native plugin. It manages the lifecycle of CUDA resources and provides
 * scoped mapping of GPU textures.
 */
class CudaInteropWrapper
{
public:
    /**
     * @brief Construct wrapper with configuration
     */
    explicit CudaInteropWrapper(const CudaInteropConfig& config = {});

    /**
     * @brief Destructor - shuts down CUDA Interop
     */
    ~CudaInteropWrapper();

    // Non-copyable
    CudaInteropWrapper(const CudaInteropWrapper&) = delete;
    CudaInteropWrapper& operator=(const CudaInteropWrapper&) = delete;

    /**
     * @brief Initialize the CUDA Interop system
     * @return true on success
     */
    bool initialize();

    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();

    /**
     * @brief Check if initialized
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief Wait for a frame from Unity
     * @param timeout_ms Timeout in milliseconds
     * @return Frame info if successful, nullopt on timeout
     */
    std::optional<CudaInteropFrameInfo> wait_for_frame(int timeout_ms);

    /**
     * @brief Map GPU resources for CUDA access
     * @return true on success
     *
     * After calling this, GPU textures can be accessed via get_cuda_array().
     * Must call unmap_resources() when done.
     */
    bool map_resources();

    /**
     * @brief Unmap GPU resources
     * @return true on success
     */
    bool unmap_resources();

    /**
     * @brief Check if resources are currently mapped
     */
    bool are_mapped() const { return mapped_; }

    /**
     * @brief Get CUDA array for a texture type
     * @param type Texture type (RGB or Depth)
     * @return cudaArray_t pointer (as void*), or nullptr on error
     */
    void* get_cuda_array(CudaInteropTextureType type);

    /**
     * @brief Get the last error message
     */
    std::string get_last_error() const;

    /**
     * @brief Get performance metrics
     */
    CudaInteropMetrics get_metrics() const;

    /**
     * @brief Reset performance metrics
     */
    void reset_metrics();

    /**
     * @brief RAII helper for resource mapping
     *
     * Usage:
     *   if (auto scope = wrapper.scoped_map()) {
     *       auto rgb = wrapper.get_cuda_array(CudaInteropTextureType::RGB);
     *       // use rgb...
     *   } // automatically unmapped
     */
    class ScopedMap
    {
    public:
        ScopedMap(CudaInteropWrapper& wrapper, bool success);
        ~ScopedMap();
        explicit operator bool() const { return success_; }

    private:
        CudaInteropWrapper& wrapper_;
        bool success_;
    };

    /**
     * @brief Map resources with RAII scope
     * @return ScopedMap that unmaps on destruction
     */
    ScopedMap scoped_map();

private:
    CudaInteropConfig config_;
    bool initialized_ = false;
    bool mapped_ = false;
    mutable std::mutex mutex_;

    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    // Function pointers for dynamically loaded plugin (if needed)
    void* plugin_handle_ = nullptr;
};

} // namespace auto_battlebot
