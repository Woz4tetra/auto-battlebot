/**
 * @file cuda_interop_wrapper.cpp
 * @brief Implementation of CUDA Interop wrapper
 */

#include "communication/cuda_interop_wrapper.hpp"

#include <dlfcn.h>
#include <cstring>

// Include the native plugin header for direct linking
// If the plugin is not available, we stub out the functions
#ifdef CUDA_INTEROP_AVAILABLE
extern "C" {
#include "cuda_interop.h"
}
#else
// Stub definitions when plugin is not available
typedef int CudaInteropError;
#define CUDA_INTEROP_SUCCESS 0
#define CUDA_INTEROP_ERROR_NOT_INITIALIZED -1
#define CUDA_INTEROP_ERROR_CUDA_NOT_AVAILABLE -3

// Stub function declarations (marked maybe_unused to avoid warnings when not all are called)
[[maybe_unused]] static CudaInteropError CudaInterop_Initialize(int) { return CUDA_INTEROP_ERROR_CUDA_NOT_AVAILABLE; }
[[maybe_unused]] static void CudaInterop_Shutdown() {}
[[maybe_unused]] static bool CudaInterop_IsInitialized() { return false; }
[[maybe_unused]] static const char* CudaInterop_GetLastError() { return "CUDA Interop not available"; }
[[maybe_unused]] static CudaInteropError CudaInterop_WaitForFrame(int, void*) { return CUDA_INTEROP_ERROR_NOT_INITIALIZED; }
[[maybe_unused]] static CudaInteropError CudaInterop_MapResources() { return CUDA_INTEROP_ERROR_NOT_INITIALIZED; }
[[maybe_unused]] static CudaInteropError CudaInterop_UnmapResources() { return CUDA_INTEROP_ERROR_NOT_INITIALIZED; }
[[maybe_unused]] static bool CudaInterop_AreMapped() { return false; }
[[maybe_unused]] static void* CudaInterop_GetCudaArray(int) { return nullptr; }
struct CudaInteropMetricsStub { double d[8]; uint64_t u[3]; };
[[maybe_unused]] static CudaInteropError CudaInterop_GetMetrics(void*) { return CUDA_INTEROP_ERROR_NOT_INITIALIZED; }
[[maybe_unused]] static void CudaInterop_ResetMetrics() {}
#endif

namespace auto_battlebot
{

CudaInteropWrapper::CudaInteropWrapper(const CudaInteropConfig& config)
    : config_(config)
{
    logger_ = DiagnosticsLogger::get_logger("cuda_interop_wrapper");
}

CudaInteropWrapper::~CudaInteropWrapper()
{
    shutdown();
}

bool CudaInteropWrapper::initialize()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_)
    {
        logger_->warning("already_initialized", "Already initialized");
        return true;
    }

#ifdef CUDA_INTEROP_AVAILABLE
    CudaInteropError err = CudaInterop_Initialize(config_.cuda_device_id);
    if (err != CUDA_INTEROP_SUCCESS)
    {
        logger_->error("initialize_failed",
            {{"error_code", err},
             {"error_msg", CudaInterop_GetLastError()}});
        return false;
    }

    initialized_ = true;
    logger_->info("initialized",
        {{"cuda_device_id", config_.cuda_device_id}});
    return true;
#else
    // When CUDA Interop is not available, we log and return false
    // The simulation can fall back to CPU-based image transfer
    logger_->warning("cuda_interop_not_available",
        {{"message", "Running without GPU texture sharing. Image transfer will use CPU."}});
    return false;
#endif
}

void CudaInteropWrapper::shutdown()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (mapped_)
    {
        unmap_resources();
    }

    if (initialized_)
    {
#ifdef CUDA_INTEROP_AVAILABLE
        CudaInterop_Shutdown();
#endif
        initialized_ = false;
        logger_->info("shutdown", "Shutdown complete");
    }
}

std::optional<CudaInteropFrameInfo> CudaInteropWrapper::wait_for_frame(int timeout_ms)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_)
    {
        return std::nullopt;
    }

#ifdef CUDA_INTEROP_AVAILABLE
    CudaInteropFrameInfo native_info;
    CudaInteropError err = CudaInterop_WaitForFrame(timeout_ms,
        reinterpret_cast<::CudaInteropFrameInfo*>(&native_info));

    if (err == CUDA_INTEROP_SUCCESS)
    {
        CudaInteropFrameInfo info;
        info.frame_id = native_info.frame_id;
        info.timestamp = native_info.timestamp;
        info.rgb_width = native_info.rgb_width;
        info.rgb_height = native_info.rgb_height;
        info.depth_width = native_info.depth_width;
        info.depth_height = native_info.depth_height;
        info.rgb_valid = native_info.rgb_valid;
        info.depth_valid = native_info.depth_valid;
        return info;
    }
    else if (err == CUDA_INTEROP_ERROR_TIMEOUT)
    {
        return std::nullopt;
    }
    else
    {
        logger_->warning("wait_for_frame_failed",
            {{"error_code", err},
             {"error_msg", CudaInterop_GetLastError()}});
        return std::nullopt;
    }
#else
    (void)timeout_ms;
    return std::nullopt;
#endif
}

bool CudaInteropWrapper::map_resources()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_)
    {
        return false;
    }

    if (mapped_)
    {
        return true;  // Already mapped
    }

#ifdef CUDA_INTEROP_AVAILABLE
    CudaInteropError err = CudaInterop_MapResources();
    if (err != CUDA_INTEROP_SUCCESS)
    {
        logger_->warning("map_resources_failed",
            {{"error_code", err},
             {"error_msg", CudaInterop_GetLastError()}});
        return false;
    }

    mapped_ = true;
    return true;
#else
    return false;
#endif
}

bool CudaInteropWrapper::unmap_resources()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!mapped_)
    {
        return true;  // Already unmapped
    }

#ifdef CUDA_INTEROP_AVAILABLE
    CudaInteropError err = CudaInterop_UnmapResources();
    mapped_ = false;  // Reset even on error

    if (err != CUDA_INTEROP_SUCCESS)
    {
        logger_->warning("unmap_resources_failed",
            {{"error_code", err},
             {"error_msg", CudaInterop_GetLastError()}});
        return false;
    }

    return true;
#else
    return false;
#endif
}

void* CudaInteropWrapper::get_cuda_array(CudaInteropTextureType type)
{
    // Note: no lock here as this should only be called while mapped
    // and mapping already holds the lock

    if (!initialized_ || !mapped_)
    {
        return nullptr;
    }

#ifdef CUDA_INTEROP_AVAILABLE
    return CudaInterop_GetCudaArray(static_cast<int>(type));
#else
    (void)type;
    return nullptr;
#endif
}

std::string CudaInteropWrapper::get_last_error() const
{
#ifdef CUDA_INTEROP_AVAILABLE
    const char* err = CudaInterop_GetLastError();
    return err ? err : "";
#else
    return "CUDA Interop not available";
#endif
}

CudaInteropMetrics CudaInteropWrapper::get_metrics() const
{
    CudaInteropMetrics metrics{};

#ifdef CUDA_INTEROP_AVAILABLE
    ::CudaInteropMetrics native_metrics;
    if (CudaInterop_GetMetrics(&native_metrics) == CUDA_INTEROP_SUCCESS)
    {
        metrics.last_map_time_ms = native_metrics.last_map_time_ms;
        metrics.last_unmap_time_ms = native_metrics.last_unmap_time_ms;
        metrics.last_sync_time_ms = native_metrics.last_sync_time_ms;
        metrics.avg_map_time_ms = native_metrics.avg_map_time_ms;
        metrics.avg_unmap_time_ms = native_metrics.avg_unmap_time_ms;
        metrics.total_frames = native_metrics.total_frames;
        metrics.map_errors = native_metrics.map_errors;
        metrics.unmap_errors = native_metrics.unmap_errors;
    }
#endif

    return metrics;
}

void CudaInteropWrapper::reset_metrics()
{
#ifdef CUDA_INTEROP_AVAILABLE
    CudaInterop_ResetMetrics();
#endif
}

// ScopedMap implementation

CudaInteropWrapper::ScopedMap::ScopedMap(CudaInteropWrapper& wrapper, bool success)
    : wrapper_(wrapper), success_(success)
{
}

CudaInteropWrapper::ScopedMap::~ScopedMap()
{
    if (success_)
    {
        wrapper_.unmap_resources();
    }
}

CudaInteropWrapper::ScopedMap CudaInteropWrapper::scoped_map()
{
    return ScopedMap(*this, map_resources());
}

} // namespace auto_battlebot
