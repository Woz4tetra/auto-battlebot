// Auto-Battlebot Simulation System
// C# bindings for the CudaInteropPlugin native library
//
// This enables zero-copy GPU texture sharing between Unity and C++
// via CUDA-OpenGL interop. Textures remain on GPU throughout the pipeline.
//
// Usage:
//   1. Call CudaInterop.Initialize() at startup
//   2. Register RenderTextures via CudaInterop.RegisterTexture()
//   3. After rendering, call CudaInterop.SyncAndNotify() or use GL.IssuePluginEvent
//   4. C++ application maps textures via CudaInterop_MapResources() native call

using System;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;

namespace AutoBattlebot.NativePlugins
{
    /// <summary>
    /// Error codes from the CUDA Interop native plugin.
    /// </summary>
    public enum CudaInteropError
    {
        Success = 0,
        NotInitialized = -1,
        AlreadyInitialized = -2,
        CudaNotAvailable = -3,
        OpenGLNotAvailable = -4,
        InvalidTexture = -5,
        TextureNotFound = -6,
        RegistrationFailed = -7,
        MapFailed = -8,
        UnmapFailed = -9,
        SyncFailed = -10,
        Timeout = -11,
        Internal = -99,
    }

    /// <summary>
    /// Texture type for CUDA Interop registration.
    /// </summary>
    public enum CudaInteropTextureType
    {
        RGB = 0,
        Depth = 1,
    }

    /// <summary>
    /// Performance metrics from the native plugin.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CudaInteropMetrics
    {
        public double LastMapTimeMs;
        public double LastUnmapTimeMs;
        public double LastSyncTimeMs;
        public double AvgMapTimeMs;
        public double AvgUnmapTimeMs;
        public ulong TotalFrames;
        public ulong MapErrors;
        public ulong UnmapErrors;
    }

    /// <summary>
    /// Frame information from the native plugin.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CudaInteropFrameInfo
    {
        public ulong FrameId;
        public double Timestamp;
        public int RgbWidth;
        public int RgbHeight;
        public int DepthWidth;
        public int DepthHeight;
        [MarshalAs(UnmanagedType.I1)]
        public bool RgbValid;
        [MarshalAs(UnmanagedType.I1)]
        public bool DepthValid;
    }

    /// <summary>
    /// C# wrapper for the CudaInteropPlugin native library.
    /// Provides zero-copy GPU texture sharing between Unity and C++ via CUDA.
    /// </summary>
    public static class CudaInterop
    {
        private const string DLL_NAME = "CudaInteropPlugin";

        #region Native Imports

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_Initialize(int deviceId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern void CudaInterop_Shutdown();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        private static extern bool CudaInterop_IsInitialized();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CudaInterop_GetLastError();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_RegisterTexture(
            ulong textureId,
            int width,
            int height,
            CudaInteropTextureType textureType);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_UnregisterTexture(ulong textureId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern void CudaInterop_UnregisterAllTextures();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_SyncAndNotify(ulong frameId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_WaitForFrame(
            int timeoutMs,
            out CudaInteropFrameInfo outFrameInfo);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_MapResources();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_UnmapResources();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        private static extern bool CudaInterop_AreMapped();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CudaInterop_GetCudaArray(CudaInteropTextureType textureType);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CudaInterop_GetCudaArrayById(ulong textureId);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern CudaInteropError CudaInterop_GetMetrics(out CudaInteropMetrics outMetrics);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern void CudaInterop_ResetMetrics();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CudaInterop_GetRenderEventFunc();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern void CudaInterop_SetNextFrameId(ulong frameId);

        #endregion

        #region Private State

        private static bool _initialized = false;
        private static bool _pluginAvailable = true;
        private static ulong _frameId = 0;
        private static IntPtr _renderEventFunc = IntPtr.Zero;

        // Registered texture tracking
        private static ulong _rgbTextureId = 0;
        private static ulong _depthTextureId = 0;

        #endregion

        #region Properties

        /// <summary>
        /// Whether the CUDA Interop system is initialized.
        /// </summary>
        public static bool IsInitialized => _initialized;

        /// <summary>
        /// Whether the native plugin is available (DLL loaded successfully).
        /// </summary>
        public static bool IsPluginAvailable => _pluginAvailable;

        /// <summary>
        /// Current frame ID.
        /// </summary>
        public static ulong FrameId => _frameId;

        /// <summary>
        /// Whether a RGB texture is registered.
        /// </summary>
        public static bool HasRgbTexture => _rgbTextureId != 0;

        /// <summary>
        /// Whether a depth texture is registered.
        /// </summary>
        public static bool HasDepthTexture => _depthTextureId != 0;

        #endregion

        #region Public API

        /// <summary>
        /// Initialize the CUDA Interop system.
        /// </summary>
        /// <param name="deviceId">CUDA device ID (default: 0)</param>
        /// <returns>True if initialization succeeded</returns>
        public static bool Initialize(int deviceId = 0)
        {
            if (_initialized)
            {
                Debug.Log("[CudaInterop] Already initialized");
                return true;
            }

            if (!_pluginAvailable)
            {
                Debug.LogWarning("[CudaInterop] Native plugin not available");
                return false;
            }

            try
            {
                var result = CudaInterop_Initialize(deviceId);

                if (result == CudaInteropError.Success)
                {
                    _initialized = true;
                    _renderEventFunc = CudaInterop_GetRenderEventFunc();
                    Debug.Log($"[CudaInterop] Initialized successfully on device {deviceId}");
                    return true;
                }
                else if (result == CudaInteropError.AlreadyInitialized)
                {
                    _initialized = true;
                    _renderEventFunc = CudaInterop_GetRenderEventFunc();
                    Debug.Log("[CudaInterop] Already initialized (native side)");
                    return true;
                }
                else
                {
                    Debug.LogError($"[CudaInterop] Initialization failed: {result} - {GetLastError()}");
                    return false;
                }
            }
            catch (DllNotFoundException e)
            {
                Debug.LogWarning($"[CudaInterop] Native plugin not found: {e.Message}");
                _pluginAvailable = false;
                return false;
            }
            catch (EntryPointNotFoundException e)
            {
                Debug.LogWarning($"[CudaInterop] Native function not found: {e.Message}");
                _pluginAvailable = false;
                return false;
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] Initialization exception: {e}");
                return false;
            }
        }

        /// <summary>
        /// Shutdown the CUDA Interop system.
        /// </summary>
        public static void Shutdown()
        {
            if (!_initialized || !_pluginAvailable)
            {
                return;
            }

            try
            {
                CudaInterop_Shutdown();
                _initialized = false;
                _rgbTextureId = 0;
                _depthTextureId = 0;
                _renderEventFunc = IntPtr.Zero;
                Debug.Log("[CudaInterop] Shutdown complete");
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] Shutdown exception: {e}");
            }
        }

        /// <summary>
        /// Get the last error message from the native plugin.
        /// </summary>
        public static string GetLastError()
        {
            if (!_pluginAvailable) return "Plugin not available";

            try
            {
                IntPtr errorPtr = CudaInterop_GetLastError();
                return errorPtr != IntPtr.Zero ? Marshal.PtrToStringAnsi(errorPtr) : "No error";
            }
            catch
            {
                return "Failed to get error";
            }
        }

        /// <summary>
        /// Register a RenderTexture for CUDA access.
        /// </summary>
        /// <param name="texture">The RenderTexture to register</param>
        /// <param name="textureType">Type of texture (RGB or Depth)</param>
        /// <returns>True if registration succeeded</returns>
        public static bool RegisterTexture(RenderTexture texture, CudaInteropTextureType textureType)
        {
            if (!_initialized)
            {
                Debug.LogError("[CudaInterop] Not initialized");
                return false;
            }

            if (texture == null)
            {
                Debug.LogError("[CudaInterop] Texture is null");
                return false;
            }

            // Ensure texture is created
            if (!texture.IsCreated())
            {
                texture.Create();
            }

            // Get native texture pointer (OpenGL texture ID)
            IntPtr nativePtr = texture.GetNativeTexturePtr();
            if (nativePtr == IntPtr.Zero)
            {
                Debug.LogError("[CudaInterop] Failed to get native texture pointer");
                return false;
            }

            ulong textureId = (ulong)nativePtr.ToInt64();

            try
            {
                var result = CudaInterop_RegisterTexture(
                    textureId,
                    texture.width,
                    texture.height,
                    textureType);

                if (result == CudaInteropError.Success)
                {
                    // Track registered texture
                    if (textureType == CudaInteropTextureType.RGB)
                    {
                        _rgbTextureId = textureId;
                    }
                    else
                    {
                        _depthTextureId = textureId;
                    }

                    Debug.Log($"[CudaInterop] Registered {textureType} texture: {texture.width}x{texture.height} (ID: {textureId})");
                    return true;
                }
                else
                {
                    Debug.LogError($"[CudaInterop] Failed to register texture: {result} - {GetLastError()}");
                    return false;
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] RegisterTexture exception: {e}");
                return false;
            }
        }

        /// <summary>
        /// Unregister a texture.
        /// </summary>
        /// <param name="textureType">Type of texture to unregister</param>
        public static void UnregisterTexture(CudaInteropTextureType textureType)
        {
            if (!_initialized) return;

            ulong textureId = textureType == CudaInteropTextureType.RGB ? _rgbTextureId : _depthTextureId;
            if (textureId == 0) return;

            try
            {
                CudaInterop_UnregisterTexture(textureId);

                if (textureType == CudaInteropTextureType.RGB)
                {
                    _rgbTextureId = 0;
                }
                else
                {
                    _depthTextureId = 0;
                }

                Debug.Log($"[CudaInterop] Unregistered {textureType} texture");
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] UnregisterTexture exception: {e}");
            }
        }

        /// <summary>
        /// Unregister all textures.
        /// </summary>
        public static void UnregisterAllTextures()
        {
            if (!_initialized) return;

            try
            {
                CudaInterop_UnregisterAllTextures();
                _rgbTextureId = 0;
                _depthTextureId = 0;
                Debug.Log("[CudaInterop] Unregistered all textures");
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] UnregisterAllTextures exception: {e}");
            }
        }

        /// <summary>
        /// Synchronize with OpenGL and notify that a frame is ready.
        /// Call this after rendering completes.
        /// </summary>
        /// <returns>True if sync succeeded</returns>
        public static bool SyncAndNotify()
        {
            if (!_initialized) return false;

            try
            {
                _frameId++;
                var result = CudaInterop_SyncAndNotify(_frameId);

                if (result != CudaInteropError.Success)
                {
                    Debug.LogWarning($"[CudaInterop] SyncAndNotify failed: {result}");
                    return false;
                }

                return true;
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] SyncAndNotify exception: {e}");
                return false;
            }
        }

        /// <summary>
        /// Issue a plugin render event via GL.IssuePluginEvent.
        /// This is the preferred way to call SyncAndNotify on the render thread.
        /// </summary>
        /// <param name="commandBuffer">Optional CommandBuffer to append the event to</param>
        public static void IssueRenderEvent(CommandBuffer commandBuffer = null)
        {
            if (!_initialized || _renderEventFunc == IntPtr.Zero) return;

            _frameId++;

            try
            {
                // Set the frame ID that will be used by the render event
                CudaInterop_SetNextFrameId(_frameId);

                if (commandBuffer != null)
                {
                    // Add to command buffer (executed on render thread)
                    commandBuffer.IssuePluginEvent(_renderEventFunc, 0);
                }
                else
                {
                    // Issue directly (must be on render thread)
                    GL.IssuePluginEvent(_renderEventFunc, 0);
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] IssueRenderEvent exception: {e}");
            }
        }

        /// <summary>
        /// Get performance metrics from the native plugin.
        /// </summary>
        /// <param name="metrics">Output: performance metrics</param>
        /// <returns>True if successful</returns>
        public static bool GetMetrics(out CudaInteropMetrics metrics)
        {
            metrics = default;

            if (!_initialized) return false;

            try
            {
                var result = CudaInterop_GetMetrics(out metrics);
                return result == CudaInteropError.Success;
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] GetMetrics exception: {e}");
                return false;
            }
        }

        /// <summary>
        /// Reset performance metrics.
        /// </summary>
        public static void ResetMetrics()
        {
            if (!_initialized) return;

            try
            {
                CudaInterop_ResetMetrics();
            }
            catch (Exception e)
            {
                Debug.LogError($"[CudaInterop] ResetMetrics exception: {e}");
            }
        }

        /// <summary>
        /// Get a formatted performance report string.
        /// </summary>
        public static string GetPerformanceReport()
        {
            if (!GetMetrics(out var metrics))
            {
                return "[CudaInterop] Metrics not available";
            }

            return $"[CudaInterop] Performance:\n" +
                   $"  Total frames: {metrics.TotalFrames}\n" +
                   $"  Last map time: {metrics.LastMapTimeMs:F3} ms\n" +
                   $"  Last unmap time: {metrics.LastUnmapTimeMs:F3} ms\n" +
                   $"  Last sync time: {metrics.LastSyncTimeMs:F3} ms\n" +
                   $"  Avg map time: {metrics.AvgMapTimeMs:F3} ms\n" +
                   $"  Avg unmap time: {metrics.AvgUnmapTimeMs:F3} ms\n" +
                   $"  Map errors: {metrics.MapErrors}\n" +
                   $"  Unmap errors: {metrics.UnmapErrors}";
        }

        #endregion
    }
}
