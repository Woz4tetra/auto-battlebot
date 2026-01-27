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
#include <cuda_gl_interop.h>
#include <GL/gl.h>
#include <GL/glext.h>
#endif

namespace {

// Per-texture CUDA resources (for CUDA backend)
struct CudaTextureResource {
    cudaGraphicsResource_t resource = nullptr;
    bool mapped = false;
    void* cuda_ptr = nullptr;
    size_t size = 0;
};

// Global state
struct GpuShareState {
    bool initialized = false;
    GpuShareBackend backend = GPU_SHARE_BACKEND_AUTO;
    std::string shared_name;
    int default_timeout_ms = 1000;
    
    // Registered textures
    std::unordered_map<uint64_t, GpuShareTexture> textures;
    std::unordered_map<std::string, uint64_t> texture_names;  // name -> handle lookup
    std::unordered_map<uint64_t, CudaTextureResource> cuda_resources;  // CUDA backend only
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
#ifdef CUDA_AVAILABLE
            std::cout << "[GpuMemoryShare] Initializing CUDA backend..." << std::endl;
            // Ensure CUDA device is set (default device 0)
            cudaError_t err = cudaSetDevice(0);
            if (err != cudaSuccess) {
                std::cerr << "[GpuMemoryShare] Failed to set CUDA device: " << cudaGetErrorString(err) << std::endl;
                return GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE;
            }
            std::cout << "[GpuMemoryShare] CUDA device 0 selected" << std::endl;
#else
            std::cerr << "[GpuMemoryShare] CUDA backend requested but CUDA not available" << std::endl;
            return GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE;
#endif
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
    
    // Cleanup registered CUDA resources
#ifdef CUDA_AVAILABLE
    for (auto& [handle, cuda_res] : g_state.cuda_resources) {
        if (cuda_res.mapped && cuda_res.resource) {
            cudaGraphicsUnmapResources(1, &cuda_res.resource, 0);
        }
        if (cuda_res.resource) {
            cudaGraphicsUnregisterResource(cuda_res.resource);
        }
    }
    g_state.cuda_resources.clear();
#endif
    
    // Cleanup backend
    switch (g_state.backend) {
        case GPU_SHARE_BACKEND_CUDA:
            // CUDA resources cleaned above
            break;
        case GPU_SHARE_BACKEND_OPENGL:
            // OpenGL cleanup (if needed)
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
#ifdef CUDA_AVAILABLE
            {
                // Unity's GetNativeTexturePtr() on Linux/OpenGL returns GLuint (texture ID)
                // Cast the void* pointer to GLuint
                GLuint gl_texture_id = reinterpret_cast<GLuint>(reinterpret_cast<uintptr_t>(info->native_texture_ptr));
                
                if (gl_texture_id == 0) {
                    std::cerr << "[GpuMemoryShare] Invalid OpenGL texture ID (0)" << std::endl;
                    return GPU_SHARE_ERROR_INVALID_TEXTURE;
                }
                
                CudaTextureResource cuda_res;
                cudaError_t err = cudaGraphicsGLRegisterImage(
                    &cuda_res.resource,
                    gl_texture_id,
                    GL_TEXTURE_2D,
                    cudaGraphicsRegisterFlagsReadOnly);
                
                if (err != cudaSuccess) {
                    std::cerr << "[GpuMemoryShare] Failed to register OpenGL texture with CUDA: " 
                              << cudaGetErrorString(err) << std::endl;
                    return GPU_SHARE_ERROR_SHARING_FAILED;
                }
                
                g_state.cuda_resources[texture.handle] = cuda_res;
                std::cout << "[GpuMemoryShare] Registered OpenGL texture " << gl_texture_id 
                          << " with CUDA (handle=" << texture.handle << ")" << std::endl;
            }
#else
            return GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE;
#endif
            break;
            
        case GPU_SHARE_BACKEND_OPENGL:
            // OpenGL backend: texture ID is already in native_texture_ptr
            // No additional registration needed for basic OpenGL sharing
            break;
            
        default:
            break;
    }
    
    g_state.textures[texture.handle] = texture;
    
    // Store name->handle mapping for lookup
    if (info->name && strlen(info->name) > 0)
    {
        g_state.texture_names[std::string(info->name)] = texture.handle;
    }
    
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
        // Backend-specific cleanup
        if (g_state.backend == GPU_SHARE_BACKEND_CUDA) {
#ifdef CUDA_AVAILABLE
            auto cuda_it = g_state.cuda_resources.find(texture->handle);
            if (cuda_it != g_state.cuda_resources.end()) {
                CudaTextureResource& cuda_res = cuda_it->second;
                if (cuda_res.mapped && cuda_res.resource) {
                    cudaGraphicsUnmapResources(1, &cuda_res.resource, 0);
                }
                if (cuda_res.resource) {
                    cudaGraphicsUnregisterResource(cuda_res.resource);
                }
                g_state.cuda_resources.erase(cuda_it);
            }
#endif
        }
        
        g_state.textures.erase(it);
        std::cout << "[GpuMemoryShare] Unregistered texture handle=" << texture->handle << std::endl;
    }
    
    texture->handle = 0;
}

GpuShareError GpuMemoryShare_GetTextureByName(
    const char* name,
    GpuShareTexture* out_texture) {
    
    std::lock_guard<std::mutex> lock(g_state_mutex);
    
    if (!g_state.initialized || !name || !out_texture) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    auto name_it = g_state.texture_names.find(std::string(name));
    if (name_it == g_state.texture_names.end()) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    auto tex_it = g_state.textures.find(name_it->second);
    if (tex_it == g_state.textures.end()) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    *out_texture = tex_it->second;
    return GPU_SHARE_SUCCESS;
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
    
#ifdef CUDA_AVAILABLE
    auto cuda_it = g_state.cuda_resources.find(texture->handle);
    if (cuda_it == g_state.cuda_resources.end() || !cuda_it->second.resource) {
        return GPU_SHARE_ERROR_INVALID_HANDLE;
    }
    
    CudaTextureResource& cuda_res = cuda_it->second;
    
    // Map the resource if not already mapped
    if (!cuda_res.mapped) {
        cudaError_t err = cudaGraphicsMapResources(1, &cuda_res.resource, 0);
        if (err != cudaSuccess) {
            std::cerr << "[GpuMemoryShare] Failed to map CUDA graphics resource: " 
                      << cudaGetErrorString(err) << std::endl;
            return GPU_SHARE_ERROR_SHARING_FAILED;
        }
        cuda_res.mapped = true;
    }
    
    // Get the device pointer
    cudaArray_t cuda_array = nullptr;
    cudaGraphicsResourceGetMappedArray(&cuda_array, cuda_res.resource);
    
    if (cuda_array == nullptr) {
        std::cerr << "[GpuMemoryShare] Failed to get mapped CUDA array" << std::endl;
        return GPU_SHARE_ERROR_SHARING_FAILED;
    }
    
    // For 2D textures, we need to get the device pointer differently
    // cudaGraphicsResourceGetMappedPointer works for buffers, not arrays
    // For arrays, we'd need to use cudaMemcpy2DFromArray or similar
    // For now, return the array pointer cast (caller must handle array access)
    *out_cuda_ptr = reinterpret_cast<void*>(cuda_array);
    
    return GPU_SHARE_SUCCESS;
#else
    return GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE;
#endif
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
    
    // Backend-specific copy
    switch (g_state.backend) {
        case GPU_SHARE_BACKEND_CUDA:
#ifdef CUDA_AVAILABLE
            {
                auto cuda_it = g_state.cuda_resources.find(texture->handle);
                if (cuda_it == g_state.cuda_resources.end() || !cuda_it->second.resource) {
                    return GPU_SHARE_ERROR_INVALID_HANDLE;
                }
                
                CudaTextureResource& cuda_res = cuda_it->second;
                
                // Map the resource if not already mapped
                if (!cuda_res.mapped) {
                    cudaError_t err = cudaGraphicsMapResources(1, &cuda_res.resource, 0);
                    if (err != cudaSuccess) {
                        std::cerr << "[GpuMemoryShare] Failed to map CUDA resource for copy: " 
                                  << cudaGetErrorString(err) << std::endl;
                        return GPU_SHARE_ERROR_SHARING_FAILED;
                    }
                    cuda_res.mapped = true;
                }
                
                // Get the mapped array
                cudaArray_t cuda_array = nullptr;
                cudaGraphicsResourceGetMappedArray(&cuda_array, cuda_res.resource);
                if (cuda_array == nullptr) {
                    return GPU_SHARE_ERROR_SHARING_FAILED;
                }
                
                // Copy from 2D array to linear buffer
                // cudaMemcpy2DFromArray copies from a CUDA array to host memory
                // Parameters: dst, dst_pitch, src_array, src_x, src_y, width, height, kind
                size_t dst_pitch = texture->width * bytes_per_pixel;
                size_t width_bytes = texture->width * bytes_per_pixel;
                
                cudaError_t err = cudaMemcpy2DFromArray(
                    dst_buffer,           // dst
                    dst_pitch,            // dst pitch (bytes per row)
                    cuda_array,           // src array
                    0, 0,                 // src offset (x, y)
                    width_bytes,          // width in bytes
                    texture->height,      // height in rows
                    cudaMemcpyDeviceToHost);
                
                if (err != cudaSuccess) {
                    std::cerr << "[GpuMemoryShare] Failed to copy from CUDA array: " 
                              << cudaGetErrorString(err) << std::endl;
                    return GPU_SHARE_ERROR_SHARING_FAILED;
                }
                
                return GPU_SHARE_SUCCESS;
            }
#else
            return GPU_SHARE_ERROR_CUDA_NOT_AVAILABLE;
#endif
            
        case GPU_SHARE_BACKEND_OPENGL:
            // OpenGL: use glGetTextureImage (requires OpenGL 4.5+)
            // For now, return not implemented
            return GPU_SHARE_ERROR_OPENGL_NOT_AVAILABLE;
            
        default:
            return GPU_SHARE_ERROR_SHARING_FAILED;
    }
}

} // extern "C"
