/**
 * @file cuda_interop.h
 * @brief CUDA Interop API for Unity Native Plugin
 *
 * This header defines the public API for zero-copy GPU texture sharing between
 * Unity (OpenGL) and the C++ application via CUDA. This eliminates CPU-GPU
 * round-trips by keeping image data on the GPU throughout the pipeline.
 *
 * Usage from Unity:
 *   1. Call CudaInterop_Initialize() once at startup
 *   2. Get RenderTexture native pointers via GetNativeTexturePtr()
 *   3. Call CudaInterop_RegisterTexture() for RGB and Depth textures
 *   4. After each frame renders, call CudaInterop_SyncAndNotify()
 *
 * Usage from C++ application:
 *   1. Call CudaInterop_WaitForFrame() to wait for Unity frame
 *   2. Call CudaInterop_MapResources() to map textures to CUDA
 *   3. Call CudaInterop_GetCudaArray() to get cudaArray_t for inference
 *   4. After processing, call CudaInterop_UnmapResources()
 *
 * @note Requires NVIDIA GPU with CUDA support and Unity using OpenGL backend
 */

#ifndef CUDA_INTEROP_H
#define CUDA_INTEROP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Platform-specific export macros
#ifdef _WIN32
    #ifdef CUDA_INTEROP_EXPORTS
        #define CUDA_INTEROP_API __declspec(dllexport)
    #else
        #define CUDA_INTEROP_API __declspec(dllimport)
    #endif
#else
    #define CUDA_INTEROP_API __attribute__((visibility("default")))
#endif

// Unity plugin interface compatibility
#define UNITY_INTERFACE_EXPORT CUDA_INTEROP_API

// ============================================================================
// Error Codes
// ============================================================================

typedef enum {
    CUDA_INTEROP_SUCCESS = 0,
    CUDA_INTEROP_ERROR_NOT_INITIALIZED = -1,
    CUDA_INTEROP_ERROR_ALREADY_INITIALIZED = -2,
    CUDA_INTEROP_ERROR_CUDA_NOT_AVAILABLE = -3,
    CUDA_INTEROP_ERROR_OPENGL_NOT_AVAILABLE = -4,
    CUDA_INTEROP_ERROR_INVALID_TEXTURE = -5,
    CUDA_INTEROP_ERROR_TEXTURE_NOT_FOUND = -6,
    CUDA_INTEROP_ERROR_REGISTRATION_FAILED = -7,
    CUDA_INTEROP_ERROR_MAP_FAILED = -8,
    CUDA_INTEROP_ERROR_UNMAP_FAILED = -9,
    CUDA_INTEROP_ERROR_SYNC_FAILED = -10,
    CUDA_INTEROP_ERROR_TIMEOUT = -11,
    CUDA_INTEROP_ERROR_INTERNAL = -99,
} CudaInteropError;

// ============================================================================
// Texture Types
// ============================================================================

typedef enum {
    CUDA_INTEROP_TEXTURE_RGB = 0,    // RGBA8 color texture
    CUDA_INTEROP_TEXTURE_DEPTH = 1,  // R32F depth texture
} CudaInteropTextureType;

// ============================================================================
// Performance Metrics
// ============================================================================

typedef struct {
    double last_map_time_ms;         // Time for last MapResources call
    double last_unmap_time_ms;       // Time for last UnmapResources call
    double last_sync_time_ms;        // Time for last GL sync fence wait
    double avg_map_time_ms;          // Running average map time
    double avg_unmap_time_ms;        // Running average unmap time
    uint64_t total_frames;           // Total frames processed
    uint64_t map_errors;             // Number of map errors
    uint64_t unmap_errors;           // Number of unmap errors
} CudaInteropMetrics;

// ============================================================================
// Frame Info
// ============================================================================

typedef struct {
    uint64_t frame_id;               // Incrementing frame counter
    double timestamp;                // Frame timestamp (seconds since init)
    int rgb_width;                   // RGB texture width
    int rgb_height;                  // RGB texture height
    int depth_width;                 // Depth texture width
    int depth_height;                // Depth texture height
    bool rgb_valid;                  // True if RGB texture is registered
    bool depth_valid;                // True if depth texture is registered
} CudaInteropFrameInfo;

// ============================================================================
// Initialization and Shutdown
// ============================================================================

/**
 * @brief Initialize the CUDA Interop system
 *
 * This must be called once before any other functions. It initializes the
 * CUDA context and prepares for OpenGL-CUDA interop.
 *
 * @param device_id CUDA device ID to use (typically 0)
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_Initialize(int device_id);

/**
 * @brief Shutdown and cleanup all resources
 *
 * Unregisters all textures and releases CUDA resources.
 */
CUDA_INTEROP_API void CudaInterop_Shutdown(void);

/**
 * @brief Check if the system is initialized
 * @return true if initialized, false otherwise
 */
CUDA_INTEROP_API bool CudaInterop_IsInitialized(void);

/**
 * @brief Get the last error message
 * @return Pointer to static error message string
 */
CUDA_INTEROP_API const char* CudaInterop_GetLastError(void);

// ============================================================================
// Texture Registration (Called from Unity)
// ============================================================================

/**
 * @brief Register a Unity OpenGL texture for CUDA access
 *
 * Call this after creating a RenderTexture in Unity. The texture_id is
 * obtained from RenderTexture.GetNativeTexturePtr().ToInt64().
 *
 * @param texture_id OpenGL texture ID from Unity
 * @param width Texture width in pixels
 * @param height Texture height in pixels
 * @param texture_type Type of texture (RGB or Depth)
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_RegisterTexture(
    uint64_t texture_id,
    int width,
    int height,
    CudaInteropTextureType texture_type);

/**
 * @brief Unregister a previously registered texture
 *
 * @param texture_id OpenGL texture ID to unregister
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_UnregisterTexture(uint64_t texture_id);

/**
 * @brief Unregister all textures
 */
CUDA_INTEROP_API void CudaInterop_UnregisterAllTextures(void);

// ============================================================================
// Frame Synchronization (Called from Unity after rendering)
// ============================================================================

/**
 * @brief Insert GL sync fence and notify that frame is ready
 *
 * This should be called from Unity after rendering completes (e.g., in
 * OnPostRender or via GL.IssuePluginEvent). It inserts a GL fence to
 * ensure all rendering is complete before CUDA access.
 *
 * @param frame_id Current frame number
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_SyncAndNotify(uint64_t frame_id);

/**
 * @brief Wait for a frame to be ready (called from C++ application)
 *
 * Blocks until Unity signals a frame is ready or timeout occurs.
 *
 * @param timeout_ms Timeout in milliseconds (0 = no wait, -1 = infinite)
 * @param out_frame_info Output: frame information
 * @return CUDA_INTEROP_SUCCESS on success, CUDA_INTEROP_ERROR_TIMEOUT on timeout
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_WaitForFrame(
    int timeout_ms,
    CudaInteropFrameInfo* out_frame_info);

// ============================================================================
// CUDA Resource Mapping (Called from C++ application)
// ============================================================================

/**
 * @brief Map all registered textures for CUDA access
 *
 * This must be called before GetCudaArray. After mapping, OpenGL cannot
 * access the textures until UnmapResources is called.
 *
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_MapResources(void);

/**
 * @brief Unmap all resources after CUDA processing
 *
 * This must be called after processing to allow OpenGL to render again.
 *
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_UnmapResources(void);

/**
 * @brief Check if resources are currently mapped
 * @return true if mapped, false otherwise
 */
CUDA_INTEROP_API bool CudaInterop_AreMapped(void);

/**
 * @brief Get the CUDA array for a registered texture
 *
 * This returns a cudaArray_t that can be used directly with CUDA kernels
 * or bound to a texture object for TensorRT inference.
 *
 * @param texture_type Which texture to get (RGB or Depth)
 * @return Pointer to cudaArray (cast to void* for C compatibility), or NULL on error
 *
 * @note Resources must be mapped before calling this function
 */
CUDA_INTEROP_API void* CudaInterop_GetCudaArray(CudaInteropTextureType texture_type);

/**
 * @brief Get CUDA array for a specific texture ID
 *
 * Alternative to GetCudaArray that uses texture ID instead of type.
 *
 * @param texture_id OpenGL texture ID
 * @return Pointer to cudaArray, or NULL on error
 */
CUDA_INTEROP_API void* CudaInterop_GetCudaArrayById(uint64_t texture_id);

// ============================================================================
// Performance Metrics
// ============================================================================

/**
 * @brief Get performance metrics
 *
 * @param out_metrics Output: performance metrics structure
 * @return CUDA_INTEROP_SUCCESS on success
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_GetMetrics(CudaInteropMetrics* out_metrics);

/**
 * @brief Reset performance metrics
 */
CUDA_INTEROP_API void CudaInterop_ResetMetrics(void);

// ============================================================================
// Unity Render Callback (for GL.IssuePluginEvent)
// ============================================================================

/**
 * @brief Get the render event callback function pointer
 *
 * This returns a function pointer that can be used with Unity's
 * GL.IssuePluginEvent() to call plugin functions on the render thread.
 *
 * Event IDs:
 *   0 = SyncAndNotify (call after rendering)
 *   1 = Reserved
 *
 * @return Function pointer for GL.IssuePluginEvent
 */
CUDA_INTEROP_API void* CudaInterop_GetRenderEventFunc(void);

#ifdef __cplusplus
}
#endif

#endif // CUDA_INTEROP_H
