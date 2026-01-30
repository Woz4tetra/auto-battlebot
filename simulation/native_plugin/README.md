# CUDA Interop Unity Native Plugin

This native plugin enables zero-copy GPU texture sharing between Unity and the C++ application using CUDA-OpenGL interop. Textures rendered by Unity remain on the GPU and are accessed directly by the C++ application's TensorRT inference pipeline.

## Requirements

- **NVIDIA GPU** with CUDA Compute Capability 5.0+ (Jetson Orin Nano: 8.7)
- **CUDA Toolkit 12.x** installed and in PATH
- **CMake 3.18+**
- **GCC/G++** with C++17 support
- **OpenGL development libraries**: `libgl1-mesa-dev`
- **Unity** must use **OpenGL graphics API** (not Vulkan or DirectX)

### Ubuntu/Debian Installation

```bash
# Install build dependencies
sudo apt-get update
sudo apt-get install -y cmake build-essential libgl1-mesa-dev

# CUDA Toolkit should be installed separately from NVIDIA
# Verify with:
nvcc --version
```

### Jetson (JetPack 6.x)

CUDA is pre-installed with JetPack. Verify with:

```bash
nvcc --version
# Should show CUDA 12.x
```

## Building

### Quick Build

```bash
cd simulation/native_plugin
./build_and_install_unity_plugin.sh
```

### Manual Build

```bash
cd simulation/native_plugin
mkdir build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_CUDA=ON \
    -DENABLE_OPENGL=ON

cmake --build . --config Release -j$(nproc)
cmake --install . --config Release
```

### Build Options

| Option                | Default | Description                                |
| --------------------- | ------- | ------------------------------------------ |
| `ENABLE_CUDA`         | ON      | Enable CUDA backend (required for interop) |
| `ENABLE_OPENGL`       | ON      | Enable OpenGL support                      |
| `BUILD_LEGACY_PLUGIN` | OFF     | Also build legacy GpuMemoryShare plugin    |
| `UNITY_PLUGINS_DIR`   | Auto    | Override Unity Plugins installation path   |

### Environment Variables

```bash
BUILD_TYPE=Debug    # Build debug version
ENABLE_CUDA=OFF     # Disable CUDA (for testing only)
BUILD_LEGACY=ON     # Also build legacy plugin
```

## Output

After building, the following files are installed:

```
auto-battlebot-sim/Assets/Plugins/
├── x86_64/
│   └── libCudaInteropPlugin.so   # Native plugin library
└── cuda_interop.h                 # C header for reference
```

## Unity Setup

### 1. Set Graphics API to OpenGL

**Edit > Project Settings > Player > Other Settings**

Under "Graphics APIs", ensure **OpenGL Core** is at the top of the list (or the only entry). Remove Vulkan if present.

### 2. Import Native Plugin

The build script automatically copies the plugin to `Assets/Plugins/x86_64/`. Unity should auto-detect it.

If Unity shows "Multiple plugins with same name" error, delete any duplicate `.so` files in the Plugins folder.

### 3. C# Usage

```csharp
using AutoBattlebot.NativePlugins;
using UnityEngine;
using UnityEngine.Rendering;

public class CameraCapture : MonoBehaviour
{
    public RenderTexture rgbTexture;
    public RenderTexture depthTexture;

    private CommandBuffer _commandBuffer;

    void Start()
    {
        // Initialize CUDA Interop
        if (!CudaInterop.Initialize())
        {
            Debug.LogError("Failed to initialize CUDA Interop");
            return;
        }

        // Register textures
        CudaInterop.RegisterTexture(rgbTexture, CudaInteropTextureType.RGB);
        CudaInterop.RegisterTexture(depthTexture, CudaInteropTextureType.Depth);

        // Set up command buffer for render event
        _commandBuffer = new CommandBuffer { name = "CUDA Interop Sync" };
        Camera.main.AddCommandBuffer(CameraEvent.AfterEverything, _commandBuffer);
    }

    void OnPostRender()
    {
        // Option 1: Direct call (must be on render thread)
        CudaInterop.SyncAndNotify();

        // Option 2: Via command buffer (preferred)
        // _commandBuffer.Clear();
        // CudaInterop.IssueRenderEvent(_commandBuffer);
    }

    void OnDestroy()
    {
        CudaInterop.UnregisterAllTextures();
        CudaInterop.Shutdown();

        if (_commandBuffer != null)
        {
            Camera.main.RemoveCommandBuffer(CameraEvent.AfterEverything, _commandBuffer);
            _commandBuffer.Dispose();
        }
    }
}
```

## C++ Application Usage

The C++ application accesses the shared textures through the same native plugin:

```cpp
#include "cuda_interop.h"
#include <cuda_runtime.h>

// Initialize (call once)
CudaInterop_Initialize(0);  // Device 0

// Main loop
while (running) {
    // Wait for Unity to signal frame ready
    CudaInteropFrameInfo frameInfo;
    auto result = CudaInterop_WaitForFrame(100, &frameInfo);  // 100ms timeout
    if (result != CUDA_INTEROP_SUCCESS) continue;

    // Map resources for CUDA access
    CudaInterop_MapResources();

    // Get CUDA arrays (these are GPU pointers)
    cudaArray_t rgbArray = (cudaArray_t)CudaInterop_GetCudaArray(CUDA_INTEROP_TEXTURE_RGB);
    cudaArray_t depthArray = (cudaArray_t)CudaInterop_GetCudaArray(CUDA_INTEROP_TEXTURE_DEPTH);

    // Use with TensorRT or other CUDA processing
    // ... process frames directly on GPU ...

    // Unmap resources (must do before next frame)
    CudaInterop_UnmapResources();
}

// Cleanup
CudaInterop_Shutdown();
```

## Architecture

```
Unity Process                     C++ Process
┌─────────────────┐              ┌─────────────────┐
│  RenderTexture  │              │  TensorRT       │
│  (OpenGL)       │              │  Inference      │
└────────┬────────┘              └────────▲────────┘
         │                                │
         │ GetNativeTexturePtr()          │ cudaArray_t
         ▼                                │
┌────────────────────────────────────────────────────┐
│              CudaInteropPlugin (Native)             │
│  ┌──────────────────────────────────────────────┐  │
│  │  cudaGraphicsGLRegisterImage()               │  │
│  │  cudaGraphicsMapResources()                  │  │
│  │  cudaGraphicsSubResourceGetMappedArray()     │  │
│  └──────────────────────────────────────────────┘  │
│                                                     │
│  ┌──────────────────────────────────────────────┐  │
│  │  Frame Sync (mutex + condition variable)     │  │
│  └──────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
         │                                │
         │ GL Sync Fence                  │ CUDA Stream
         ▼                                ▼
┌─────────────────────────────────────────────────────┐
│                    GPU Memory                        │
│  (Texture data never leaves GPU)                    │
└─────────────────────────────────────────────────────┘
```

## Performance

Target metrics (per frame):

- **Map/Unmap cycle**: < 0.5 ms
- **GL Sync fence wait**: < 1 ms
- **Total overhead**: < 2 ms

Use `CudaInterop.GetPerformanceReport()` in Unity or `CudaInterop_GetMetrics()` in C++ to monitor performance.

## Troubleshooting

### "CUDA not available" error

1. Verify CUDA is installed: `nvcc --version`
2. Check GPU is detected: `nvidia-smi`
3. Ensure CUDA libraries are in `LD_LIBRARY_PATH`

### "OpenGL not available" error

1. Verify Unity is using OpenGL (check Player Settings)
2. Check OpenGL is working: `glxinfo | grep "OpenGL version"`

### "Registration failed" error

1. Ensure RenderTexture is created before registering
2. Check texture format is supported (RGBA32, R32F)
3. Verify texture has `antiAliasing = 1` (no MSAA)

### Unity crashes on startup

1. Check for duplicate plugins in Assets/Plugins
2. Verify plugin architecture matches Unity (x86_64)
3. Check `ldd libCudaInteropPlugin.so` for missing dependencies

### Frame tearing or corruption

1. Ensure GL sync fence is working (check sync time in metrics)
2. Verify Unmap is called before Unity renders next frame
3. Check for concurrent access issues

## License

Part of the Auto-Battlebot project.
