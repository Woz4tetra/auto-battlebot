/**
 * @file gpu_memory_share.h
 * @brief Zero-copy GPU memory sharing between Unity and C++
 * 
 * This plugin enables direct GPU memory sharing to eliminate the GPU→CPU→SharedMem→CPU
 * pipeline, reducing latency from ~200ms to ~10ms.
 * 
 * Supported backends:
 * - CUDA: For NVIDIA GPUs with CUDA support in both Unity and C++
 * - OpenGL: Using GL_EXT_memory_object for cross-process sharing
 * - Vulkan: Using VK_KHR_external_memory (future)
 * 
 * Usage from Unity:
 *   1. Call GpuMemoryShare_Initialize() at startup
 *   2. Call GpuMemoryShare_RegisterTexture() for each texture to share
 *   3. Call GpuMemoryShare_NotifyFrameReady() after rendering
 *   4. C++ side calls GpuMemoryShare_WaitForFrame() then reads directly from GPU
 * 
 * Usage from C++:
 *   1. Call GpuMemoryShare_OpenShared() with the same identifiers
 *   2. Call GpuMemoryShare_WaitForFrame() to sync
 *   3. Access GPU memory directly via CUDA/OpenGL/Vulkan
 */

#ifndef GPU_MEMORY_SHARE_H
#define GPU_MEMORY_SHARE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Export macros for cross-platform DLL/SO
#ifdef _WIN32
    #define GPU_SHARE_EXPORT __declspec(dllexport)
#else
    #define GPU_SHARE_EXPORT __attribute__((visibility("default")))
#endif

// Error codes
typedef enum {
    GPU_SHARE_SUCCESS = 0,
    GPU_SHARE_ERROR_NOT_INITIALIZED = -1,
    GPU_SHARE_ERROR_INVALID_TEXTURE = -2,
    GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE = -3,
    GPU_SHARE_ERROR_OPENGL_NOT_AVAILABLE = -4,
    GPU_SHARE_ERROR_SHARING_FAILED = -5,
    GPU_SHARE_ERROR_TIMEOUT = -6,
    GPU_SHARE_ERROR_INVALID_HANDLE = -7,
} GpuShareError;

// Backend type
typedef enum {
    GPU_SHARE_BACKEND_AUTO = 0,  // Auto-detect best backend
    GPU_SHARE_BACKEND_CUDA = 1,
    GPU_SHARE_BACKEND_OPENGL = 2,
    GPU_SHARE_BACKEND_VULKAN = 3,
} GpuShareBackend;

// Texture format
typedef enum {
    GPU_SHARE_FORMAT_RGBA8 = 0,
    GPU_SHARE_FORMAT_BGRA8 = 1,
    GPU_SHARE_FORMAT_R32F = 2,   // Single channel float (for depth)
    GPU_SHARE_FORMAT_RGB8 = 3,
} GpuShareFormat;

// Configuration for initialization
typedef struct {
    GpuShareBackend backend;
    const char* shared_name;     // Name for shared memory region
    int timeout_ms;              // Default timeout for wait operations
} GpuShareConfig;

// Texture registration info
typedef struct {
    void* native_texture_ptr;    // Unity's native texture pointer
    int width;
    int height;
    GpuShareFormat format;
    const char* name;            // Identifier for this texture
} GpuShareTextureInfo;

// Shared texture handle (returned from registration)
typedef struct {
    uint64_t handle;             // Opaque handle
    int width;
    int height;
    GpuShareFormat format;
} GpuShareTexture;

// Frame info passed with each frame
typedef struct {
    uint64_t frame_id;
    double timestamp;
    bool has_color;
    bool has_depth;
} GpuShareFrameInfo;

/**
 * @brief Initialize the GPU memory sharing system
 * @param config Configuration options
 * @return GPU_SHARE_SUCCESS or error code
 */
GPU_SHARE_EXPORT GpuShareError GpuMemoryShare_Initialize(const GpuShareConfig* config);

/**
 * @brief Shutdown and cleanup
 */
GPU_SHARE_EXPORT void GpuMemoryShare_Shutdown(void);

/**
 * @brief Check if initialized
 */
GPU_SHARE_EXPORT bool GpuMemoryShare_IsInitialized(void);

/**
 * @brief Get the active backend
 */
GPU_SHARE_EXPORT GpuShareBackend GpuMemoryShare_GetBackend(void);

/**
 * @brief Register a Unity texture for sharing
 * @param info Texture information
 * @param out_texture Output: shared texture handle
 * @return GPU_SHARE_SUCCESS or error code
 */
GPU_SHARE_EXPORT GpuShareError GpuMemoryShare_RegisterTexture(
    const GpuShareTextureInfo* info,
    GpuShareTexture* out_texture);

/**
 * @brief Unregister a shared texture
 * @param texture Texture handle
 */
GPU_SHARE_EXPORT void GpuMemoryShare_UnregisterTexture(GpuShareTexture* texture);

/**
 * @brief Notify that a frame is ready (call after Unity renders)
 * @param frame_info Frame metadata
 * @return GPU_SHARE_SUCCESS or error code
 */
GPU_SHARE_EXPORT GpuShareError GpuMemoryShare_NotifyFrameReady(
    const GpuShareFrameInfo* frame_info);

/**
 * @brief Wait for a frame to be ready (call from C++ consumer)
 * @param timeout_ms Timeout in milliseconds (-1 for default)
 * @param out_frame_info Output: frame metadata
 * @return GPU_SHARE_SUCCESS or error code
 */
GPU_SHARE_EXPORT GpuShareError GpuMemoryShare_WaitForFrame(
    int timeout_ms,
    GpuShareFrameInfo* out_frame_info);

/**
 * @brief Get CUDA device pointer for a shared texture (CUDA backend only)
 * @param texture Texture handle
 * @param out_cuda_ptr Output: CUDA device pointer
 * @return GPU_SHARE_SUCCESS or error code
 */
GPU_SHARE_EXPORT GpuShareError GpuMemoryShare_GetCudaPtr(
    const GpuShareTexture* texture,
    void** out_cuda_ptr);

/**
 * @brief Get OpenGL texture ID for a shared texture (OpenGL backend only)
 * @param texture Texture handle
 * @param out_gl_texture Output: OpenGL texture ID
 * @return GPU_SHARE_SUCCESS or error code
 */
GPU_SHARE_EXPORT GpuShareError GpuMemoryShare_GetOpenGLTexture(
    const GpuShareTexture* texture,
    uint32_t* out_gl_texture);

/**
 * @brief Copy from shared texture to CPU buffer (fallback)
 * @param texture Texture handle
 * @param dst_buffer Destination CPU buffer
 * @param buffer_size Size of destination buffer
 * @return GPU_SHARE_SUCCESS or error code
 */
GPU_SHARE_EXPORT GpuShareError GpuMemoryShare_CopyToCpu(
    const GpuShareTexture* texture,
    void* dst_buffer,
    size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // GPU_MEMORY_SHARE_H
