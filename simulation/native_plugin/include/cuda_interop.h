/**
 * @file cuda_interop.h
 * @brief CUDA Interop API for Unity Native Plugin
 *
 * This header defines the public API for zero-copy GPU texture sharing between
 * Unity (OpenGL or Vulkan) and the C++ application via CUDA. This eliminates
 * CPU-GPU round-trips by keeping image data on the GPU throughout the pipeline.
 *
 * Supported Graphics APIs:
 *   - OpenGL: Uses cudaGraphicsGLRegisterImage for texture registration
 *   - Vulkan: Uses cudaImportExternalMemory for texture registration
 *
 * Usage from Unity:
 *   1. Call CudaInterop_InitializeWithAPI() once at startup with detected graphics API
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
 * @note Requires NVIDIA GPU with CUDA support
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
// Graphics API Types
// ============================================================================

typedef enum {
    CUDA_INTEROP_GRAPHICS_API_UNKNOWN = 0,
    CUDA_INTEROP_GRAPHICS_API_OPENGL = 1,
    CUDA_INTEROP_GRAPHICS_API_VULKAN = 2,
} CudaInteropGraphicsAPI;

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
    CUDA_INTEROP_ERROR_VULKAN_NOT_AVAILABLE = -12,
    CUDA_INTEROP_ERROR_UNSUPPORTED_GRAPHICS_API = -13,
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
 * @brief Initialize the CUDA Interop system with specific graphics API
 *
 * This must be called once before any other functions. It initializes the
 * CUDA context and prepares for interop with the specified graphics API.
 *
 * @param device_id CUDA device ID to use (typically 0)
 * @param graphics_api The graphics API Unity is using (OpenGL or Vulkan)
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_InitializeWithAPI(
    int device_id,
    CudaInteropGraphicsAPI graphics_api);

/**
 * @brief Initialize the CUDA Interop system (legacy, defaults to OpenGL)
 *
 * This is a convenience wrapper that calls CudaInterop_InitializeWithAPI
 * with CUDA_INTEROP_GRAPHICS_API_OPENGL for backward compatibility.
 *
 * @param device_id CUDA device ID to use (typically 0)
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 * @deprecated Use CudaInterop_InitializeWithAPI instead
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_Initialize(int device_id);

/**
 * @brief Get the currently active graphics API
 * @return The graphics API that was used during initialization
 */
CUDA_INTEROP_API CudaInteropGraphicsAPI CudaInterop_GetGraphicsAPI(void);

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
 * @brief Register a Unity texture for CUDA access
 *
 * Call this after creating a RenderTexture in Unity. The texture_id is
 * obtained from RenderTexture.GetNativeTexturePtr().ToInt64().
 *
 * The meaning of texture_id depends on the graphics API:
 *   - OpenGL: GL texture name (GLuint)
 *   - Vulkan: VkImage handle
 *
 * @param texture_id Native texture handle from Unity
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
 * @brief Register a Vulkan texture with additional memory info
 *
 * For Vulkan, this extended registration function provides the memory
 * handle required for CUDA external memory import.
 *
 * @param texture_id VkImage handle from Unity
 * @param width Texture width in pixels
 * @param height Texture height in pixels
 * @param texture_type Type of texture (RGB or Depth)
 * @param memory_handle Platform-specific memory handle (fd on Linux, HANDLE on Windows)
 * @param memory_size Total size of the Vulkan memory allocation
 * @param is_dedicated True if the memory is a dedicated allocation for this image
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_RegisterTextureVulkan(
    uint64_t texture_id,
    int width,
    int height,
    CudaInteropTextureType texture_type,
    int memory_handle,
    uint64_t memory_size,
    bool is_dedicated);

/**
 * @brief Unregister a previously registered texture
 *
 * @param texture_id Native texture handle to unregister
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
 * @brief Synchronize graphics pipeline and notify that frame is ready
 *
 * This should be called from Unity after rendering completes (e.g., in
 * OnPostRender or via GL.IssuePluginEvent). It performs synchronization
 * appropriate for the active graphics API:
 *   - OpenGL: Inserts a GL fence
 *   - Vulkan: Uses external semaphores or CUDA stream sync
 *
 * @param frame_id Current frame number
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_SyncAndNotify(uint64_t frame_id);

/**
 * @brief Register a Vulkan semaphore for synchronization
 *
 * For Vulkan, this allows importing an external semaphore that Unity
 * signals after rendering. CUDA will wait on this semaphore before
 * accessing textures.
 *
 * @param semaphore_handle Platform-specific semaphore handle (fd on Linux)
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_RegisterVulkanSemaphore(int semaphore_handle);

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
 * This must be called before GetCudaArray. After mapping, the graphics API
 * cannot access the textures until UnmapResources is called.
 *
 * Behavior by graphics API:
 *   - OpenGL: Uses cudaGraphicsMapResources
 *   - Vulkan: Resources are always accessible (no-op, returns success)
 *
 * @return CUDA_INTEROP_SUCCESS on success, error code otherwise
 */
CUDA_INTEROP_API CudaInteropError CudaInterop_MapResources(void);

/**
 * @brief Unmap all resources after CUDA processing
 *
 * This must be called after processing to allow the graphics API to render again.
 *
 * Behavior by graphics API:
 *   - OpenGL: Uses cudaGraphicsUnmapResources
 *   - Vulkan: Signals completion (may signal external semaphore)
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
