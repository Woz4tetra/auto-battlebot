/**
 * @file cuda_interop.cu
 * @brief CUDA Interop implementation for Unity Native Plugin
 *
 * Implements zero-copy GPU texture sharing between Unity (OpenGL or Vulkan)
 * and the C++ application using CUDA graphics interop.
 *
 * Supported backends:
 *   - OpenGL: Uses cudaGraphicsGLRegisterImage
 *   - Vulkan: Uses cudaImportExternalMemory with Vulkan external memory
 */

#include "cuda_interop.h"

#include <cuda_runtime.h>

// OpenGL support (conditionally compiled)
#ifndef DISABLE_OPENGL
#include <cuda_gl_interop.h>
#include <GL/gl.h>

// OpenGL sync extension types and constants (GL 3.2+)
// We define these here to avoid header compatibility issues with nvcc
#ifndef GL_SYNC_GPU_COMMANDS_COMPLETE
#define GL_SYNC_GPU_COMMANDS_COMPLETE     0x9117
#endif
#ifndef GL_SYNC_FLUSH_COMMANDS_BIT
#define GL_SYNC_FLUSH_COMMANDS_BIT        0x00000001
#endif
#ifndef GL_TIMEOUT_EXPIRED
#define GL_TIMEOUT_EXPIRED                0x911B
#endif
#ifndef GL_WAIT_FAILED
#define GL_WAIT_FAILED                    0x911D
#endif

// GL sync object type
typedef struct __GLsync* GLsync;

// Function pointer types for GL sync functions (loaded at runtime)
typedef GLsync (*PFNGLFENCESYNCPROC)(GLenum condition, GLbitfield flags);
typedef void (*PFNGLDELETESYNCPROC)(GLsync sync);
typedef GLenum (*PFNGLCLIENTWAITSYNCPROC)(GLsync sync, GLbitfield flags, uint64_t timeout);
#endif // DISABLE_OPENGL

// Vulkan support (conditionally compiled)
#ifdef ENABLE_VULKAN
#include <vulkan/vulkan.h>
#endif

// We'll load these dynamically
#include <dlfcn.h>

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>
#include <cstring>

// Unity plugin interface macro
#ifndef UNITY_INTERFACE_API
    #ifdef _WIN32
        #define UNITY_INTERFACE_API __stdcall
    #else
        #define UNITY_INTERFACE_API
    #endif
#endif

// ============================================================================
// Internal Data Structures
// ============================================================================

namespace {

/**
 * @brief Information about a registered texture
 */
struct RegisteredTexture {
    uint64_t native_handle;              // OpenGL texture ID or VkImage
    int width;
    int height;
    CudaInteropTextureType type;
    
    // OpenGL resources (used when graphics_api == OPENGL)
    cudaGraphicsResource_t cuda_graphics_resource; // CUDA graphics resource handle
    
    // Vulkan resources (used when graphics_api == VULKAN)
    cudaExternalMemory_t cuda_external_memory;     // CUDA external memory handle
    cudaMipmappedArray_t cuda_mipmap_array;        // Mipmapped array from external memory
    
    // Common
    cudaArray_t cuda_array;               // Mapped CUDA array (valid when mapped)
    bool is_registered;
    bool is_mapped;
    
    // Vulkan-specific info
    int vk_memory_handle;                 // File descriptor for Vulkan memory
    uint64_t vk_memory_size;              // Size of Vulkan memory allocation
    bool vk_is_dedicated;                 // Whether allocation is dedicated
};

/**
 * @brief Global state for the CUDA Interop system
 */
struct CudaInteropState {
    // Initialization state
    std::atomic<bool> initialized{false};
    int cuda_device_id = 0;
    CudaInteropGraphicsAPI graphics_api = CUDA_INTEROP_GRAPHICS_API_UNKNOWN;

    // Registered textures
    std::mutex texture_mutex;
    std::unordered_map<uint64_t, RegisteredTexture> textures;
    RegisteredTexture* rgb_texture = nullptr;    // Quick access to RGB
    RegisteredTexture* depth_texture = nullptr;  // Quick access to Depth

    // Mapping state
    std::atomic<bool> resources_mapped{false};

    // Frame synchronization
    std::mutex frame_mutex;
    std::condition_variable frame_cv;
    std::atomic<uint64_t> current_frame_id{0};
    std::atomic<bool> frame_ready{false};
    double frame_timestamp = 0.0;

#ifndef DISABLE_OPENGL
    // OpenGL sync
    GLsync gl_sync_fence = nullptr;
    bool gl_sync_available = false;
    PFNGLFENCESYNCPROC glFenceSync_ptr = nullptr;
    PFNGLDELETESYNCPROC glDeleteSync_ptr = nullptr;
    PFNGLCLIENTWAITSYNCPROC glClientWaitSync_ptr = nullptr;
#endif

#ifdef ENABLE_VULKAN
    // Vulkan sync
    cudaExternalSemaphore_t vk_semaphore = nullptr;
    bool vk_semaphore_available = false;
#endif

    // Timing
    std::chrono::steady_clock::time_point init_time;

    // Performance metrics
    CudaInteropMetrics metrics{};
    std::mutex metrics_mutex;

    // Error state
    std::string last_error;
    std::mutex error_mutex;
};

CudaInteropState g_state;

#ifndef DISABLE_OPENGL
// Helper to load GL sync functions
void load_gl_sync_functions() {
    // Try to load GL sync functions
    void* gl_lib = dlopen("libGL.so.1", RTLD_LAZY | RTLD_NOLOAD);
    if (!gl_lib) {
        gl_lib = dlopen("libGL.so", RTLD_LAZY);
    }
    
    if (gl_lib) {
        g_state.glFenceSync_ptr = (PFNGLFENCESYNCPROC)dlsym(gl_lib, "glFenceSync");
        g_state.glDeleteSync_ptr = (PFNGLDELETESYNCPROC)dlsym(gl_lib, "glDeleteSync");
        g_state.glClientWaitSync_ptr = (PFNGLCLIENTWAITSYNCPROC)dlsym(gl_lib, "glClientWaitSync");
        
        g_state.gl_sync_available = (g_state.glFenceSync_ptr != nullptr &&
                                      g_state.glDeleteSync_ptr != nullptr &&
                                      g_state.glClientWaitSync_ptr != nullptr);
        
        if (g_state.gl_sync_available) {
            std::cout << "[CudaInterop] GL sync functions loaded successfully" << std::endl;
        } else {
            std::cout << "[CudaInterop] GL sync functions not available, using CUDA sync" << std::endl;
        }
        // Don't close gl_lib - we need the symbols to remain valid
    } else {
        std::cout << "[CudaInterop] Could not load libGL, using CUDA sync" << std::endl;
    }
}
#endif // DISABLE_OPENGL

// Get graphics API name for logging
const char* get_graphics_api_name(CudaInteropGraphicsAPI api) {
    switch (api) {
        case CUDA_INTEROP_GRAPHICS_API_OPENGL: return "OpenGL";
        case CUDA_INTEROP_GRAPHICS_API_VULKAN: return "Vulkan";
        default: return "Unknown";
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

void set_error(const std::string& error) {
    std::lock_guard<std::mutex> lock(g_state.error_mutex);
    g_state.last_error = error;
    std::cerr << "[CudaInterop] ERROR: " << error << std::endl;
}

void log_info(const std::string& message) {
    std::cout << "[CudaInterop] " << message << std::endl;
}

double get_elapsed_seconds() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration<double>(now - g_state.init_time);
    return duration.count();
}

const char* cuda_error_string(cudaError_t error) {
    return cudaGetErrorString(error);
}

/**
 * @brief Check CUDA error and set error message
 */
bool check_cuda_error(cudaError_t error, const char* operation) {
    if (error != cudaSuccess) {
        std::string msg = std::string(operation) + " failed: " + cuda_error_string(error);
        set_error(msg);
        return false;
    }
    return true;
}

/**
 * @brief Get current time in milliseconds for timing
 */
double get_time_ms() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(now.time_since_epoch()).count();
}

/**
 * @brief Update running average metric
 */
void update_avg(double& avg, double new_value, uint64_t count) {
    if (count == 0) {
        avg = new_value;
    } else {
        // Exponential moving average with alpha = 0.1
        avg = avg * 0.9 + new_value * 0.1;
    }
}

} // anonymous namespace

// ============================================================================
// Public API Implementation
// ============================================================================

extern "C" {

CudaInteropError CudaInterop_InitializeWithAPI(int device_id, CudaInteropGraphicsAPI graphics_api) {
    if (g_state.initialized.load()) {
        log_info("Already initialized");
        return CUDA_INTEROP_ERROR_ALREADY_INITIALIZED;
    }

    log_info("Initializing CUDA Interop with " + std::string(get_graphics_api_name(graphics_api)) + " backend...");

    // Validate graphics API
    if (graphics_api == CUDA_INTEROP_GRAPHICS_API_UNKNOWN) {
        set_error("Unknown graphics API specified");
        return CUDA_INTEROP_ERROR_UNSUPPORTED_GRAPHICS_API;
    }

#ifdef DISABLE_OPENGL
    if (graphics_api == CUDA_INTEROP_GRAPHICS_API_OPENGL) {
        set_error("OpenGL support is disabled in this build");
        return CUDA_INTEROP_ERROR_OPENGL_NOT_AVAILABLE;
    }
#endif

#ifndef ENABLE_VULKAN
    if (graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
        set_error("Vulkan support is not enabled in this build");
        return CUDA_INTEROP_ERROR_VULKAN_NOT_AVAILABLE;
    }
#endif

    // Check CUDA availability
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    if (err != cudaSuccess || device_count == 0) {
        set_error("No CUDA devices found");
        return CUDA_INTEROP_ERROR_CUDA_NOT_AVAILABLE;
    }

    // Validate device ID
    if (device_id < 0 || device_id >= device_count) {
        set_error("Invalid CUDA device ID: " + std::to_string(device_id));
        return CUDA_INTEROP_ERROR_CUDA_NOT_AVAILABLE;
    }

    // Set CUDA device
    err = cudaSetDevice(device_id);
    if (!check_cuda_error(err, "cudaSetDevice")) {
        return CUDA_INTEROP_ERROR_CUDA_NOT_AVAILABLE;
    }

    // Get device properties for logging
    cudaDeviceProp props;
    cudaGetDeviceProperties(&props, device_id);
    log_info("Using CUDA device " + std::to_string(device_id) + ": " + props.name);
    log_info("  Compute capability: " + std::to_string(props.major) + "." + std::to_string(props.minor));
    log_info("  Total memory: " + std::to_string(props.totalGlobalMem / (1024 * 1024)) + " MB");

    // Initialize backend-specific resources
    if (graphics_api == CUDA_INTEROP_GRAPHICS_API_OPENGL) {
#ifndef DISABLE_OPENGL
        // Load GL sync functions for proper synchronization
        load_gl_sync_functions();
#endif
    } else if (graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
#ifdef ENABLE_VULKAN
        log_info("Vulkan backend: Using CUDA external memory interop");
        // Vulkan initialization is done per-texture via RegisterTextureVulkan
#endif
    }

    // Initialize state
    g_state.cuda_device_id = device_id;
    g_state.graphics_api = graphics_api;
    g_state.init_time = std::chrono::steady_clock::now();
    g_state.current_frame_id.store(0);
    g_state.frame_ready.store(false);
    g_state.resources_mapped.store(false);

    // Reset metrics
    std::memset(&g_state.metrics, 0, sizeof(CudaInteropMetrics));

    g_state.initialized.store(true);
    log_info("CUDA Interop initialized successfully with " + std::string(get_graphics_api_name(graphics_api)) + " backend");

    return CUDA_INTEROP_SUCCESS;
}

CudaInteropError CudaInterop_Initialize(int device_id) {
    // Legacy function - defaults to OpenGL for backward compatibility
    return CudaInterop_InitializeWithAPI(device_id, CUDA_INTEROP_GRAPHICS_API_OPENGL);
}

CudaInteropGraphicsAPI CudaInterop_GetGraphicsAPI() {
    return g_state.graphics_api;
}

void CudaInterop_Shutdown() {
    if (!g_state.initialized.load()) {
        return;
    }

    log_info("Shutting down CUDA Interop...");

    // Unmap resources if mapped
    if (g_state.resources_mapped.load()) {
        CudaInterop_UnmapResources();
    }

    // Unregister all textures
    CudaInterop_UnregisterAllTextures();

#ifndef DISABLE_OPENGL
    // Delete GL sync fence if exists
    if (g_state.gl_sync_fence && g_state.glDeleteSync_ptr) {
        g_state.glDeleteSync_ptr(g_state.gl_sync_fence);
        g_state.gl_sync_fence = nullptr;
    }
#endif

#ifdef ENABLE_VULKAN
    // Destroy Vulkan semaphore if exists
    if (g_state.vk_semaphore) {
        cudaDestroyExternalSemaphore(g_state.vk_semaphore);
        g_state.vk_semaphore = nullptr;
    }
#endif

    // Reset CUDA device (optional, helps with cleanup)
    cudaDeviceReset();

    g_state.graphics_api = CUDA_INTEROP_GRAPHICS_API_UNKNOWN;
    g_state.initialized.store(false);
    log_info("CUDA Interop shutdown complete");
}

bool CudaInterop_IsInitialized() {
    return g_state.initialized.load();
}

const char* CudaInterop_GetLastError() {
    std::lock_guard<std::mutex> lock(g_state.error_mutex);
    return g_state.last_error.c_str();
}

CudaInteropError CudaInterop_RegisterTexture(
    uint64_t texture_id,
    int width,
    int height,
    CudaInteropTextureType texture_type) {

    if (!g_state.initialized.load()) {
        set_error("Not initialized");
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    if (texture_id == 0 || width <= 0 || height <= 0) {
        set_error("Invalid texture parameters");
        return CUDA_INTEROP_ERROR_INVALID_TEXTURE;
    }

    // For Vulkan, require the extended registration function
    if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
        set_error("Vulkan requires CudaInterop_RegisterTextureVulkan with memory handle");
        return CUDA_INTEROP_ERROR_INVALID_TEXTURE;
    }

#ifdef DISABLE_OPENGL
    set_error("OpenGL support is disabled");
    return CUDA_INTEROP_ERROR_OPENGL_NOT_AVAILABLE;
#else
    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    // Check if already registered
    if (g_state.textures.find(texture_id) != g_state.textures.end()) {
        log_info("Texture " + std::to_string(texture_id) + " already registered, re-registering");
        // Unregister first
        auto& tex = g_state.textures[texture_id];
        if (tex.is_registered) {
            cudaGraphicsUnregisterResource(tex.cuda_graphics_resource);
        }
        g_state.textures.erase(texture_id);
    }

    // Create texture entry
    RegisteredTexture tex = {};
    tex.native_handle = texture_id;
    tex.width = width;
    tex.height = height;
    tex.type = texture_type;
    tex.cuda_graphics_resource = nullptr;
    tex.cuda_array = nullptr;
    tex.is_registered = false;
    tex.is_mapped = false;

    // Register OpenGL texture with CUDA
    // Note: GL_TEXTURE_2D is the target for standard 2D textures
    cudaError_t err = cudaGraphicsGLRegisterImage(
        &tex.cuda_graphics_resource,
        static_cast<GLuint>(texture_id),
        GL_TEXTURE_2D,
        cudaGraphicsRegisterFlagsReadOnly  // We only read from Unity's textures
    );

    if (!check_cuda_error(err, "cudaGraphicsGLRegisterImage")) {
        return CUDA_INTEROP_ERROR_REGISTRATION_FAILED;
    }

    tex.is_registered = true;

    // Store texture
    g_state.textures[texture_id] = tex;

    // Update quick access pointers
    if (texture_type == CUDA_INTEROP_TEXTURE_RGB) {
        g_state.rgb_texture = &g_state.textures[texture_id];
    } else if (texture_type == CUDA_INTEROP_TEXTURE_DEPTH) {
        g_state.depth_texture = &g_state.textures[texture_id];
    }

    const char* type_str = (texture_type == CUDA_INTEROP_TEXTURE_RGB) ? "RGB" : "Depth";
    log_info("Registered OpenGL " + std::string(type_str) + " texture: ID=" +
             std::to_string(texture_id) + " (" + std::to_string(width) + "x" +
             std::to_string(height) + ")");

    return CUDA_INTEROP_SUCCESS;
#endif // DISABLE_OPENGL
}

CudaInteropError CudaInterop_RegisterTextureVulkan(
    uint64_t texture_id,
    int width,
    int height,
    CudaInteropTextureType texture_type,
    int memory_handle,
    uint64_t memory_size,
    bool is_dedicated) {

    if (!g_state.initialized.load()) {
        set_error("Not initialized");
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    if (g_state.graphics_api != CUDA_INTEROP_GRAPHICS_API_VULKAN) {
        set_error("RegisterTextureVulkan requires Vulkan backend");
        return CUDA_INTEROP_ERROR_UNSUPPORTED_GRAPHICS_API;
    }

#ifndef ENABLE_VULKAN
    set_error("Vulkan support is not enabled in this build");
    return CUDA_INTEROP_ERROR_VULKAN_NOT_AVAILABLE;
#else
    if (texture_id == 0 || width <= 0 || height <= 0 || memory_handle < 0) {
        set_error("Invalid texture parameters");
        return CUDA_INTEROP_ERROR_INVALID_TEXTURE;
    }

    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    // Check if already registered
    if (g_state.textures.find(texture_id) != g_state.textures.end()) {
        log_info("Texture " + std::to_string(texture_id) + " already registered, re-registering");
        // Unregister first (cleanup old resources)
        auto& old_tex = g_state.textures[texture_id];
        if (old_tex.cuda_mipmap_array) {
            cudaFreeMipmappedArray(old_tex.cuda_mipmap_array);
        }
        if (old_tex.cuda_external_memory) {
            cudaDestroyExternalMemory(old_tex.cuda_external_memory);
        }
        g_state.textures.erase(texture_id);
    }

    // Create texture entry
    RegisteredTexture tex = {};
    tex.native_handle = texture_id;
    tex.width = width;
    tex.height = height;
    tex.type = texture_type;
    tex.vk_memory_handle = memory_handle;
    tex.vk_memory_size = memory_size;
    tex.vk_is_dedicated = is_dedicated;
    tex.is_registered = false;
    tex.is_mapped = false;

    // Import external memory from Vulkan
    cudaExternalMemoryHandleDesc memDesc = {};
    memDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;
    memDesc.handle.fd = memory_handle;
    memDesc.size = memory_size;
    if (is_dedicated) {
        memDesc.flags = cudaExternalMemoryDedicated;
    }

    cudaError_t err = cudaImportExternalMemory(&tex.cuda_external_memory, &memDesc);
    if (!check_cuda_error(err, "cudaImportExternalMemory")) {
        return CUDA_INTEROP_ERROR_REGISTRATION_FAILED;
    }

    // Get mipmapped array from external memory
    cudaExternalMemoryMipmappedArrayDesc arrayDesc = {};
    arrayDesc.offset = 0;
    arrayDesc.formatDesc = cudaCreateChannelDesc(
        texture_type == CUDA_INTEROP_TEXTURE_RGB ? 8 : 32,  // bits per channel
        texture_type == CUDA_INTEROP_TEXTURE_RGB ? 8 : 0,
        texture_type == CUDA_INTEROP_TEXTURE_RGB ? 8 : 0,
        texture_type == CUDA_INTEROP_TEXTURE_RGB ? 8 : 0,
        texture_type == CUDA_INTEROP_TEXTURE_RGB ? cudaChannelFormatKindUnsigned : cudaChannelFormatKindFloat
    );
    arrayDesc.extent.width = width;
    arrayDesc.extent.height = height;
    arrayDesc.extent.depth = 0;  // 2D array
    arrayDesc.flags = 0;
    arrayDesc.numLevels = 1;

    err = cudaExternalMemoryGetMappedMipmappedArray(&tex.cuda_mipmap_array, tex.cuda_external_memory, &arrayDesc);
    if (!check_cuda_error(err, "cudaExternalMemoryGetMappedMipmappedArray")) {
        cudaDestroyExternalMemory(tex.cuda_external_memory);
        return CUDA_INTEROP_ERROR_REGISTRATION_FAILED;
    }

    // Get the first mip level as the cuda array
    err = cudaGetMipmappedArrayLevel(&tex.cuda_array, tex.cuda_mipmap_array, 0);
    if (!check_cuda_error(err, "cudaGetMipmappedArrayLevel")) {
        cudaFreeMipmappedArray(tex.cuda_mipmap_array);
        cudaDestroyExternalMemory(tex.cuda_external_memory);
        return CUDA_INTEROP_ERROR_REGISTRATION_FAILED;
    }

    tex.is_registered = true;
    tex.is_mapped = true;  // Vulkan textures are always "mapped" once registered

    // Store texture
    g_state.textures[texture_id] = tex;

    // Update quick access pointers
    if (texture_type == CUDA_INTEROP_TEXTURE_RGB) {
        g_state.rgb_texture = &g_state.textures[texture_id];
    } else if (texture_type == CUDA_INTEROP_TEXTURE_DEPTH) {
        g_state.depth_texture = &g_state.textures[texture_id];
    }

    const char* type_str = (texture_type == CUDA_INTEROP_TEXTURE_RGB) ? "RGB" : "Depth";
    log_info("Registered Vulkan " + std::string(type_str) + " texture: ID=" +
             std::to_string(texture_id) + " (" + std::to_string(width) + "x" +
             std::to_string(height) + ")");

    return CUDA_INTEROP_SUCCESS;
#endif // ENABLE_VULKAN
}

CudaInteropError CudaInterop_UnregisterTexture(uint64_t texture_id) {
    if (!g_state.initialized.load()) {
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    auto it = g_state.textures.find(texture_id);
    if (it == g_state.textures.end()) {
        return CUDA_INTEROP_ERROR_TEXTURE_NOT_FOUND;
    }

    RegisteredTexture& tex = it->second;

    if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_OPENGL) {
#ifndef DISABLE_OPENGL
        // Unmap if mapped
        if (tex.is_mapped) {
            cudaGraphicsUnmapResources(1, &tex.cuda_graphics_resource, 0);
            tex.is_mapped = false;
        }

        // Unregister from CUDA
        if (tex.is_registered) {
            cudaError_t err = cudaGraphicsUnregisterResource(tex.cuda_graphics_resource);
            if (err != cudaSuccess) {
                log_info("Warning: cudaGraphicsUnregisterResource failed: " +
                         std::string(cuda_error_string(err)));
            }
            tex.is_registered = false;
        }
#endif
    } else if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
#ifdef ENABLE_VULKAN
        // Cleanup Vulkan resources
        if (tex.cuda_mipmap_array) {
            cudaFreeMipmappedArray(tex.cuda_mipmap_array);
            tex.cuda_mipmap_array = nullptr;
        }
        if (tex.cuda_external_memory) {
            cudaDestroyExternalMemory(tex.cuda_external_memory);
            tex.cuda_external_memory = nullptr;
        }
        tex.is_registered = false;
        tex.is_mapped = false;
#endif
    }

    // Clear quick access pointers if this was RGB or Depth
    if (g_state.rgb_texture == &tex) {
        g_state.rgb_texture = nullptr;
    }
    if (g_state.depth_texture == &tex) {
        g_state.depth_texture = nullptr;
    }

    g_state.textures.erase(it);

    log_info("Unregistered texture: ID=" + std::to_string(texture_id));
    return CUDA_INTEROP_SUCCESS;
}

void CudaInterop_UnregisterAllTextures() {
    if (!g_state.initialized.load()) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    for (auto& pair : g_state.textures) {
        RegisteredTexture& tex = pair.second;

        if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_OPENGL) {
#ifndef DISABLE_OPENGL
            if (tex.is_mapped) {
                cudaGraphicsUnmapResources(1, &tex.cuda_graphics_resource, 0);
            }
            if (tex.is_registered) {
                cudaGraphicsUnregisterResource(tex.cuda_graphics_resource);
            }
#endif
        } else if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
#ifdef ENABLE_VULKAN
            if (tex.cuda_mipmap_array) {
                cudaFreeMipmappedArray(tex.cuda_mipmap_array);
            }
            if (tex.cuda_external_memory) {
                cudaDestroyExternalMemory(tex.cuda_external_memory);
            }
#endif
        }
    }

    g_state.textures.clear();
    g_state.rgb_texture = nullptr;
    g_state.depth_texture = nullptr;

    log_info("Unregistered all textures");
}

CudaInteropError CudaInterop_RegisterVulkanSemaphore(int semaphore_handle) {
#ifndef ENABLE_VULKAN
    set_error("Vulkan support is not enabled in this build");
    return CUDA_INTEROP_ERROR_VULKAN_NOT_AVAILABLE;
#else
    if (!g_state.initialized.load()) {
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    if (g_state.graphics_api != CUDA_INTEROP_GRAPHICS_API_VULKAN) {
        set_error("RegisterVulkanSemaphore requires Vulkan backend");
        return CUDA_INTEROP_ERROR_UNSUPPORTED_GRAPHICS_API;
    }

    // Destroy old semaphore if exists
    if (g_state.vk_semaphore) {
        cudaDestroyExternalSemaphore(g_state.vk_semaphore);
        g_state.vk_semaphore = nullptr;
    }

    // Import external semaphore
    cudaExternalSemaphoreHandleDesc semDesc = {};
    semDesc.type = cudaExternalSemaphoreHandleTypeOpaqueFd;
    semDesc.handle.fd = semaphore_handle;
    semDesc.flags = 0;

    cudaError_t err = cudaImportExternalSemaphore(&g_state.vk_semaphore, &semDesc);
    if (!check_cuda_error(err, "cudaImportExternalSemaphore")) {
        return CUDA_INTEROP_ERROR_SYNC_FAILED;
    }

    g_state.vk_semaphore_available = true;
    log_info("Vulkan semaphore registered successfully");
    return CUDA_INTEROP_SUCCESS;
#endif
}

CudaInteropError CudaInterop_SyncAndNotify(uint64_t frame_id) {
    if (!g_state.initialized.load()) {
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    double start_time = get_time_ms();

    if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_OPENGL) {
#ifndef DISABLE_OPENGL
        // Use GL sync if available, otherwise use CUDA device synchronize
        if (g_state.gl_sync_available) {
            // Delete old sync fence if exists
            if (g_state.gl_sync_fence && g_state.glDeleteSync_ptr) {
                g_state.glDeleteSync_ptr(g_state.gl_sync_fence);
                g_state.gl_sync_fence = nullptr;
            }

            // Create new sync fence
            // This ensures all OpenGL commands before this point complete before CUDA access
            g_state.gl_sync_fence = g_state.glFenceSync_ptr(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
            if (!g_state.gl_sync_fence) {
                set_error("Failed to create GL sync fence");
                return CUDA_INTEROP_ERROR_SYNC_FAILED;
            }

            // Flush to ensure fence is submitted
            glFlush();

            // Wait for the fence (with timeout to avoid deadlock)
            GLenum result = g_state.glClientWaitSync_ptr(g_state.gl_sync_fence, GL_SYNC_FLUSH_COMMANDS_BIT, 16000000); // 16ms timeout
            if (result == GL_TIMEOUT_EXPIRED) {
                log_info("Warning: GL sync fence timeout");
            } else if (result == GL_WAIT_FAILED) {
                set_error("GL sync fence wait failed");
                return CUDA_INTEROP_ERROR_SYNC_FAILED;
            }
        } else {
            // Fallback: use CUDA device synchronize
            // This is less efficient but ensures GPU operations complete
            glFinish();  // Wait for OpenGL to complete
            cudaError_t err = cudaDeviceSynchronize();
            if (err != cudaSuccess) {
                set_error("CUDA device synchronize failed: " + std::string(cuda_error_string(err)));
                return CUDA_INTEROP_ERROR_SYNC_FAILED;
            }
        }
#endif
    } else if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
#ifdef ENABLE_VULKAN
        // For Vulkan, use external semaphore if available, otherwise just sync CUDA
        if (g_state.vk_semaphore_available && g_state.vk_semaphore) {
            // Wait on the Vulkan semaphore
            cudaExternalSemaphoreWaitParams waitParams = {};
            waitParams.params.fence.value = 0;
            waitParams.flags = 0;

            cudaError_t err = cudaWaitExternalSemaphoresAsync(&g_state.vk_semaphore, &waitParams, 1, 0);
            if (err != cudaSuccess) {
                log_info("Warning: cudaWaitExternalSemaphoresAsync failed, falling back to device sync");
                cudaDeviceSynchronize();
            }
        } else {
            // Fallback: just use CUDA device synchronize
            // This assumes Unity has already completed rendering before calling this
            cudaError_t err = cudaDeviceSynchronize();
            if (err != cudaSuccess) {
                set_error("CUDA device synchronize failed: " + std::string(cuda_error_string(err)));
                return CUDA_INTEROP_ERROR_SYNC_FAILED;
            }
        }
#endif
    } else {
        // Unknown graphics API - just do a basic CUDA sync
        cudaDeviceSynchronize();
    }

    double sync_time = get_time_ms() - start_time;

    // Update metrics
    {
        std::lock_guard<std::mutex> lock(g_state.metrics_mutex);
        g_state.metrics.last_sync_time_ms = sync_time;
    }

    // Update frame info and notify waiters
    {
        std::lock_guard<std::mutex> lock(g_state.frame_mutex);
        g_state.current_frame_id.store(frame_id);
        g_state.frame_timestamp = get_elapsed_seconds();
        g_state.frame_ready.store(true);
    }
    g_state.frame_cv.notify_all();

    return CUDA_INTEROP_SUCCESS;
}

CudaInteropError CudaInterop_WaitForFrame(int timeout_ms, CudaInteropFrameInfo* out_frame_info) {
    if (!g_state.initialized.load()) {
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    std::unique_lock<std::mutex> lock(g_state.frame_mutex);

    bool success;
    if (timeout_ms < 0) {
        // Infinite wait
        g_state.frame_cv.wait(lock, []() { return g_state.frame_ready.load(); });
        success = true;
    } else if (timeout_ms == 0) {
        // No wait
        success = g_state.frame_ready.load();
    } else {
        // Timed wait
        success = g_state.frame_cv.wait_for(
            lock,
            std::chrono::milliseconds(timeout_ms),
            []() { return g_state.frame_ready.load(); }
        );
    }

    if (!success) {
        return CUDA_INTEROP_ERROR_TIMEOUT;
    }

    // Fill out frame info
    if (out_frame_info) {
        out_frame_info->frame_id = g_state.current_frame_id.load();
        out_frame_info->timestamp = g_state.frame_timestamp;

        std::lock_guard<std::mutex> tex_lock(g_state.texture_mutex);

        if (g_state.rgb_texture) {
            out_frame_info->rgb_width = g_state.rgb_texture->width;
            out_frame_info->rgb_height = g_state.rgb_texture->height;
            out_frame_info->rgb_valid = g_state.rgb_texture->is_registered;
        } else {
            out_frame_info->rgb_width = 0;
            out_frame_info->rgb_height = 0;
            out_frame_info->rgb_valid = false;
        }

        if (g_state.depth_texture) {
            out_frame_info->depth_width = g_state.depth_texture->width;
            out_frame_info->depth_height = g_state.depth_texture->height;
            out_frame_info->depth_valid = g_state.depth_texture->is_registered;
        } else {
            out_frame_info->depth_width = 0;
            out_frame_info->depth_height = 0;
            out_frame_info->depth_valid = false;
        }
    }

    // Reset frame ready flag
    g_state.frame_ready.store(false);

    return CUDA_INTEROP_SUCCESS;
}

CudaInteropError CudaInterop_MapResources() {
    if (!g_state.initialized.load()) {
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    if (g_state.resources_mapped.load()) {
        // Already mapped - this is okay, just return success
        return CUDA_INTEROP_SUCCESS;
    }

    double start_time = get_time_ms();

    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_OPENGL) {
#ifndef DISABLE_OPENGL
        // Collect all registered resources
        std::vector<cudaGraphicsResource_t> resources;
        for (auto& pair : g_state.textures) {
            if (pair.second.is_registered && !pair.second.is_mapped) {
                resources.push_back(pair.second.cuda_graphics_resource);
            }
        }

        if (resources.empty()) {
            log_info("No textures to map");
            return CUDA_INTEROP_SUCCESS;
        }

        // Map all resources at once (more efficient than one-by-one)
        cudaError_t err = cudaGraphicsMapResources(
            static_cast<int>(resources.size()),
            resources.data(),
            0  // default stream
        );

        if (!check_cuda_error(err, "cudaGraphicsMapResources")) {
            std::lock_guard<std::mutex> metrics_lock(g_state.metrics_mutex);
            g_state.metrics.map_errors++;
            return CUDA_INTEROP_ERROR_MAP_FAILED;
        }

        // Get CUDA arrays for each mapped resource
        for (auto& pair : g_state.textures) {
            RegisteredTexture& tex = pair.second;
            if (tex.is_registered) {
                err = cudaGraphicsSubResourceGetMappedArray(
                    &tex.cuda_array,
                    tex.cuda_graphics_resource,
                    0,  // array index
                    0   // mip level
                );

                if (err != cudaSuccess) {
                    set_error("Failed to get mapped array for texture " +
                              std::to_string(tex.native_handle) + ": " +
                              cuda_error_string(err));
                    // Continue anyway - other textures might work
                }

                tex.is_mapped = true;
            }
        }
#endif
    } else if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
        // For Vulkan, textures are already mapped when registered
        // Just mark them as mapped
        for (auto& pair : g_state.textures) {
            if (pair.second.is_registered) {
                pair.second.is_mapped = true;
            }
        }
    }

    g_state.resources_mapped.store(true);

    double map_time = get_time_ms() - start_time;

    // Update metrics
    {
        std::lock_guard<std::mutex> metrics_lock(g_state.metrics_mutex);
        g_state.metrics.last_map_time_ms = map_time;
        update_avg(g_state.metrics.avg_map_time_ms, map_time, g_state.metrics.total_frames);
    }

    return CUDA_INTEROP_SUCCESS;
}

CudaInteropError CudaInterop_UnmapResources() {
    if (!g_state.initialized.load()) {
        return CUDA_INTEROP_ERROR_NOT_INITIALIZED;
    }

    if (!g_state.resources_mapped.load()) {
        // Not mapped - this is okay
        return CUDA_INTEROP_SUCCESS;
    }

    double start_time = get_time_ms();

    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_OPENGL) {
#ifndef DISABLE_OPENGL
        // Collect all mapped resources
        std::vector<cudaGraphicsResource_t> resources;
        for (auto& pair : g_state.textures) {
            if (pair.second.is_mapped) {
                resources.push_back(pair.second.cuda_graphics_resource);
            }
        }

        if (!resources.empty()) {
            cudaError_t err = cudaGraphicsUnmapResources(
                static_cast<int>(resources.size()),
                resources.data(),
                0  // default stream
            );

            if (!check_cuda_error(err, "cudaGraphicsUnmapResources")) {
                std::lock_guard<std::mutex> metrics_lock(g_state.metrics_mutex);
                g_state.metrics.unmap_errors++;
                return CUDA_INTEROP_ERROR_UNMAP_FAILED;
            }
        }

        // Clear mapped state for OpenGL
        for (auto& pair : g_state.textures) {
            pair.second.is_mapped = false;
            pair.second.cuda_array = nullptr;
        }
#endif
    } else if (g_state.graphics_api == CUDA_INTEROP_GRAPHICS_API_VULKAN) {
        // For Vulkan, textures remain accessible; just update state
        // The cuda_array pointers remain valid
        // We might signal a semaphore here if we had one for signaling back to Vulkan
    }

    g_state.resources_mapped.store(false);

    double unmap_time = get_time_ms() - start_time;

    // Update metrics
    {
        std::lock_guard<std::mutex> metrics_lock(g_state.metrics_mutex);
        g_state.metrics.last_unmap_time_ms = unmap_time;
        update_avg(g_state.metrics.avg_unmap_time_ms, unmap_time, g_state.metrics.total_frames);
        g_state.metrics.total_frames++;
    }

    return CUDA_INTEROP_SUCCESS;
}

bool CudaInterop_AreMapped() {
    return g_state.resources_mapped.load();
}

void* CudaInterop_GetCudaArray(CudaInteropTextureType texture_type) {
    if (!g_state.initialized.load() || !g_state.resources_mapped.load()) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    RegisteredTexture* tex = nullptr;
    if (texture_type == CUDA_INTEROP_TEXTURE_RGB) {
        tex = g_state.rgb_texture;
    } else if (texture_type == CUDA_INTEROP_TEXTURE_DEPTH) {
        tex = g_state.depth_texture;
    }

    if (tex && tex->is_mapped) {
        return static_cast<void*>(tex->cuda_array);
    }

    return nullptr;
}

void* CudaInterop_GetCudaArrayById(uint64_t texture_id) {
    if (!g_state.initialized.load() || !g_state.resources_mapped.load()) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lock(g_state.texture_mutex);

    auto it = g_state.textures.find(texture_id);
    if (it != g_state.textures.end() && it->second.is_mapped) {
        return static_cast<void*>(it->second.cuda_array);
    }

    return nullptr;
}

CudaInteropError CudaInterop_GetMetrics(CudaInteropMetrics* out_metrics) {
    if (!out_metrics) {
        return CUDA_INTEROP_ERROR_INTERNAL;
    }

    std::lock_guard<std::mutex> lock(g_state.metrics_mutex);
    *out_metrics = g_state.metrics;
    return CUDA_INTEROP_SUCCESS;
}

void CudaInterop_ResetMetrics() {
    std::lock_guard<std::mutex> lock(g_state.metrics_mutex);
    std::memset(&g_state.metrics, 0, sizeof(CudaInteropMetrics));
}

// ============================================================================
// Unity Render Event Callback
// ============================================================================

// Frame ID passed via GL.IssuePluginEvent userData
static uint64_t s_next_frame_id = 0;

/**
 * @brief Set the frame ID for the next render event
 * Called from Unity before GL.IssuePluginEvent
 */
CUDA_INTEROP_API void CudaInterop_SetNextFrameId(uint64_t frame_id) {
    s_next_frame_id = frame_id;
}

/**
 * @brief Render event callback for Unity
 * Called on the render thread via GL.IssuePluginEvent
 */
static void UNITY_INTERFACE_API OnRenderEvent(int event_id) {
    switch (event_id) {
        case 0:
            // Sync and notify
            CudaInterop_SyncAndNotify(s_next_frame_id);
            break;
        default:
            break;
    }
}

typedef void (UNITY_INTERFACE_API *UnityRenderingEvent)(int eventId);

void* CudaInterop_GetRenderEventFunc() {
    return reinterpret_cast<void*>(static_cast<UnityRenderingEvent>(OnRenderEvent));
}

} // extern "C"
