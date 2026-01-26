/**
 * @file gpu_memory_share.cpp
 * @brief Implementation of zero-copy GPU memory sharing
 * 
 * This is a foundation implementation. Full GPU sharing requires:
 * - CUDA: Link against CUDA runtime, use cudaGraphicsResource
 * - OpenGL: Use GL_EXT_memory_object, glCreateMemoryObjectsEXT
 * - Vulkan: Use VK_KHR_external_memory
 */

#include "gpu_memory_share.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstring>

#ifdef CUDA_AVAILABLE
#include <cuda_runtime.h>
#endif

namespace {

// Global state
struct GpuShareState {
    bool initialized = false;
    GpuShareBackend backend = GPU_SHARE_BACKEND_AUTO;
    std::string shared_name;
    int default_timeout_ms = 1000;
    
    // Registered textures
    std::unordered_map<uint64_t, GpuShareTexture> textures;
    uint64_t next_handle = 1;
    
    // Frame synchronization
    std::mutex frame_mutex;
    std::condition_variable frame_cv;
    GpuShareFrameInfo latest_frame;
    bool frame_ready = false;
};

GpuShareState g_state;
std::mutex g_state_mutex;

GpuShareBackend detect_best_backend() {
    // Try CUDA first (best for NVIDIA)
#ifdef CUDA_AVAILABLE
    int cuda_device_count = 0;
    if (cudaGetDeviceCount(&cuda_device_count) == cudaSuccess && cuda_device_count > 0) {
        std::cout << "[GpuMemoryShare] CUDA backend available (" << cuda_device_count << " devices)" << std::endl;
        return GPU_SHARE_BACKEND_CUDA;
    }
#endif

    // Fall back to OpenGL
    std::cout << "[GpuMemoryShare] Using OpenGL backend (CUDA not available)" << std::endl;
    return GPU_SHARE_BACKEND_OPENGL;
}

} // anonymous namespace

extern "C" {

GpuShareError GpuMemoryShare_Initialize(const GpuShareConfig* config) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (g_state.initialized) {
        std::cout << "[GpuMemoryShare] Already initialized" << std::endl;
        return GPU_SHARE_SUCCESS;
    }
    
    if (config) {
        g_state.backend = config->backend;
        if (config->shared_name) {
            g_state.shared_name = config->shared_name;
        }
        if (config->timeout_ms > 0) {
            g_state.default_timeout_ms = config->timeout_ms;
        }
    }
    
    // Auto-detect backend if not specified
    if (g_state.backend == GPU_SHARE_BACKEND_AUTO) {
        g_state.backend = detect_best_backend();
    }
    
    // Initialize the selected backend
    switch (g_state.backend) {
        case GPU_SHARE_BACKEND_CUDA:
            std::cout << "[GpuMemoryShare] Initializing CUDA backend..." << std::endl;
            // TODO: Initialize CUDA context sharing
            // cudaSetDevice(0);
            // cudaDeviceEnablePeerAccess(...)
            break;
            
        case GPU_SHARE_BACKEND_OPENGL:
            std::cout << "[GpuMemoryShare] Initializing OpenGL backend..." << std::endl;
            // TODO: Initialize OpenGL sharing
            // Create shared context, memory objects, etc.
            break;
            
        case GPU_SHARE_BACKEND_VULKAN:
            std::cout << "[GpuMemoryShare] Vulkan backend not yet implemented" << std::endl;
            return GPU_SHARE_ERROR_OPENGL_NOT_AVAILABLE;
            
        default:
            return GPU_SHARE_ERROR_NOT_INITIALIZED;
    }
    
    g_state.initialized = true;
    std::cout << "[GpuMemoryShare] Initialized successfully" << std::endl;
    return GPU_SHARE_SUCCESS;
}

void GpuMemoryShare_Shutdown() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (!g_state.initialized) {
        return;
    }
    
    // Cleanup registered textures
    g_state.textures.clear();
    
    // Cleanup backend
    switch (g_state.backend) {
        case GPU_SHARE_BACKEND_CUDA:
            // TODO: Cleanup CUDA resources
            break;
        case GPU_SHARE_BACKEND_OPENGL:
            // TODO: Cleanup OpenGL resources
            break;
        default:
            break;
    }
    
    g_state.initialized = false;
    std::cout << "[GpuMemoryShare] Shutdown complete" << std::endl;
}

bool GpuMemoryShare_IsInitialized() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    return g_state.initialized;
}

GpuShareBackend GpuMemoryShare_GetBackend() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    return g_state.backend;
}

GpuShareError GpuMemoryShare_RegisterTexture(
    const GpuShareTextureInfo* info,
    GpuShareTexture* out_texture) {
    
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (!g_state.initialized) {
        return GPU_SHARE_ERROR_NOT_INITIALIZED;
    }
    
    if (!info || !out_texture) {
        return GPU_SHARE_ERROR_INVALID_TEXTURE;
    }
    
    // Create texture handle
    GpuShareTexture texture;
    texture.handle = g_state.next_handle++;
    texture.width = info->width;
    texture.height = info->height;
    texture.format = info->format;
    
    std::cout << "[GpuMemoryShare] Registering texture: " 
              << (info->name ? info->name : "unnamed")
              << " (" << info->width << "x" << info->height << ")"
              << " handle=" << texture.handle << std::endl;
    
    // Backend-specific registration
    switch (g_state.backend) {
        case GPU_SHARE_BACKEND_CUDA:
            // TODO: Register Unity texture with CUDA
            // cudaGraphicsGLRegisterImage(&resource, glTextureId, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsReadOnly);
            break;
            
        case GPU_SHARE_BACKEND_OPENGL:
            // TODO: Create shared OpenGL texture
            // Use EXT_memory_object to create exportable memory
            break;
            
        default:
            break;
    }
    
    g_state.textures[texture.handle] = texture;
    *out_texture = texture;
    
    return GPU_SHARE_SUCCESS;
}

void GpuMemoryShare_UnregisterTexture(GpuShareTexture* texture) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (!g_state.initialized || !texture) {
        return;
    }
    
    auto it = g_state.textures.find(texture->handle);
    if (it != g_state.textures.end()) {
        // TODO: Backend-specific cleanup
        g_state.textures.erase(it);
        std::cout << "[GpuMemoryShare] Unregistered texture handle=" << texture->handle << std::endl;
    }
    
    texture->handle = 0;
}

GpuShareError GpuMemoryShare_NotifyFrameReady(const GpuShareFrameInfo* frame_info) {
    if (!frame_info) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    {
        std::lock_guard<std::mutex> lock(g_state.frame_mutex);
        g_state.latest_frame = *frame_info;
        g_state.frame_ready = true;
    }
    
    g_state.frame_cv.notify_all();
    return GPU_SHARE_SUCCESS;
}

GpuShareError GpuMemoryShare_WaitForFrame(int timeout_ms, GpuShareFrameInfo* out_frame_info) {
    if (timeout_ms < 0) {
        timeout_ms = g_state.default_timeout_ms;
    }
    
    std::unique_lock<std::mutex> lock(g_state.frame_mutex);
    
    bool success = g_state.frame_cv.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        []() { return g_state.frame_ready; });
    
    if (!success) {
        return GPU_SHARE_ERROR_TIMEOUT;
    }
    
    if (out_frame_info) {
        *out_frame_info = g_state.latest_frame;
    }
    
    g_state.frame_ready = false;
    return GPU_SHARE_SUCCESS;
}

GpuShareError GpuMemoryShare_GetCudaPtr(
    const GpuShareTexture* texture,
    void** out_cuda_ptr) {
    
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (!g_state.initialized) {
        return GPU_SHARE_ERROR_NOT_INITIALIZED;
    }
    
    if (g_state.backend != GPU_SHARE_BACKEND_CUDA) {
        return GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE;
    }
    
    if (!texture || !out_cuda_ptr) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    // TODO: Map CUDA graphics resource and return device pointer
    // cudaGraphicsMapResources(1, &resource, 0);
    // cudaGraphicsResourceGetMappedPointer(out_cuda_ptr, &size, resource);
    
    *out_cuda_ptr = nullptr;
    return GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE;
}

GpuShareError GpuMemoryShare_GetOpenGLTexture(
    const GpuShareTexture* texture,
    uint32_t* out_gl_texture) {
    
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (!g_state.initialized) {
        return GPU_SHARE_ERROR_NOT_INITIALIZED;
    }
    
    if (g_state.backend != GPU_SHARE_BACKEND_OPENGL) {
        return GPU_SHARE_ERROR_OPENGL_NOT_AVAILABLE;
    }
    
    if (!texture || !out_gl_texture) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    // TODO: Return shared OpenGL texture ID
    *out_gl_texture = 0;
    return GPU_SHARE_ERROR_OPENGL_NOT_AVAILABLE;
}

GpuShareError GpuMemoryShare_CopyToCpu(
    const GpuShareTexture* texture,
    void* dst_buffer,
    size_t buffer_size) {
    
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (!g_state.initialized) {
        return GPU_SHARE_ERROR_NOT_INITIALIZED;
    }
    
    if (!texture || !dst_buffer) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    // Calculate expected size
    size_t bytes_per_pixel = 4;  // RGBA8
    switch (texture->format) {
        case GPU_SHARE_FORMAT_RGBA8:
        case GPU_SHARE_FORMAT_BGRA8:
            bytes_per_pixel = 4;
            break;
        case GPU_SHARE_FORMAT_RGB8:
            bytes_per_pixel = 3;
            break;
        case GPU_SHARE_FORMAT_R32F:
            bytes_per_pixel = 4;
            break;
    }
    
    size_t expected_size = texture->width * texture->height * bytes_per_pixel;
    if (buffer_size < expected_size) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    // TODO: Copy from GPU to CPU
    // For CUDA: cudaMemcpy(dst_buffer, cuda_ptr, expected_size, cudaMemcpyDeviceToHost);
    // For OpenGL: glGetTextureImage(gl_texture, 0, format, type, buffer_size, dst_buffer);
    
    return GPU_SHARE_ERROR_SHARING_FAILED;
}

} // extern "C"
