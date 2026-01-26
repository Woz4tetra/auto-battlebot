// Auto-Battlebot Simulation System
// C# bindings for the GpuMemoryShare native plugin
// 
// This enables zero-copy GPU memory sharing between Unity and C++
// for ultra-low latency frame transfer (~10ms instead of ~200ms)

using System;
using System.Runtime.InteropServices;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace AutoBattlebot.NativePlugins
{
    /// <summary>
    /// Error codes from the native plugin
    /// </summary>
    public enum GpuShareError
    {
        Success = 0,
        NotInitialized = -1,
        InvalidTexture = -2,
        CudaNotAvailable = -3,
        OpenGLNotAvailable = -4,
        SharingFailed = -5,
        Timeout = -6,
        InvalidHandle = -7,
    }

    /// <summary>
    /// GPU sharing backend type
    /// </summary>
    public enum GpuShareBackend
    {
        Auto = 0,
        Cuda = 1,
        OpenGL = 2,
        Vulkan = 3,
    }

    /// <summary>
    /// Texture format for sharing
    /// </summary>
    public enum GpuShareFormat
    {
        RGBA8 = 0,
        BGRA8 = 1,
        R32F = 2,
        RGB8 = 3,
    }

    /// <summary>
    /// Configuration for initialization
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuShareConfig
    {
        public GpuShareBackend Backend;
        public IntPtr SharedName;
        public int TimeoutMs;

        public static GpuShareConfig Default => new GpuShareConfig
        {
            Backend = GpuShareBackend.Auto,
            SharedName = IntPtr.Zero,
            TimeoutMs = 1000,
        };
    }

    /// <summary>
    /// Texture registration info
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuShareTextureInfo
    {
        public IntPtr NativeTexturePtr;
        public int Width;
        public int Height;
        public GpuShareFormat Format;
        public IntPtr Name;
    }

    /// <summary>
    /// Shared texture handle
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuShareTexture
    {
        public ulong Handle;
        public int Width;
        public int Height;
        public GpuShareFormat Format;
    }

    /// <summary>
    /// Frame info passed with each frame
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct GpuShareFrameInfo
    {
        public ulong FrameId;
        public double Timestamp;
        [MarshalAs(UnmanagedType.I1)]
        public bool HasColor;
        [MarshalAs(UnmanagedType.I1)]
        public bool HasDepth;
    }

    /// <summary>
    /// C# wrapper for the GpuMemoryShare native plugin.
    /// Provides zero-copy GPU memory sharing between Unity and C++.
    /// </summary>
    public static class GpuMemoryShare
    {
        private const string DLL_NAME = "GpuMemoryShare";

        #region Native Imports

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern GpuShareError GpuMemoryShare_Initialize(ref GpuShareConfig config);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern void GpuMemoryShare_Shutdown();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern bool GpuMemoryShare_IsInitialized();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern GpuShareBackend GpuMemoryShare_GetBackend();

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern GpuShareError GpuMemoryShare_RegisterTexture(
            ref GpuShareTextureInfo info,
            out GpuShareTexture outTexture);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern void GpuMemoryShare_UnregisterTexture(ref GpuShareTexture texture);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        private static extern GpuShareError GpuMemoryShare_NotifyFrameReady(ref GpuShareFrameInfo frameInfo);

        #endregion

        #region Public API

        private static bool _initialized = false;

        /// <summary>
        /// Initialize the GPU memory sharing system.
        /// </summary>
        /// <param name="backend">Preferred backend (Auto recommended)</param>
        /// <param name="timeoutMs">Default timeout for wait operations</param>
        /// <returns>True if initialization succeeded</returns>
        public static bool Initialize(GpuShareBackend backend = GpuShareBackend.Auto, int timeoutMs = 1000)
        {
            if (_initialized)
            {
                Debug.Log("[GpuMemoryShare] Already initialized");
                return true;
            }

            var config = new GpuShareConfig
            {
                Backend = backend,
                SharedName = IntPtr.Zero,
                TimeoutMs = timeoutMs,
            };

            try
            {
                var result = GpuMemoryShare_Initialize(ref config);
                _initialized = result == GpuShareError.Success;

                if (_initialized)
                {
                    var actualBackend = GpuMemoryShare_GetBackend();
                    Debug.Log($"[GpuMemoryShare] Initialized with backend: {actualBackend}");
                }
                else
                {
                    Debug.LogError($"[GpuMemoryShare] Initialization failed: {result}");
                }

                return _initialized;
            }
            catch (DllNotFoundException)
            {
                Debug.LogWarning("[GpuMemoryShare] Native plugin not found. GPU sharing disabled.");
                return false;
            }
            catch (Exception e)
            {
                Debug.LogError($"[GpuMemoryShare] Initialization error: {e.Message}");
                return false;
            }
        }

        /// <summary>
        /// Shutdown the GPU memory sharing system.
        /// </summary>
        public static void Shutdown()
        {
            if (!_initialized)
            {
                return;
            }

            try
            {
                GpuMemoryShare_Shutdown();
                _initialized = false;
                Debug.Log("[GpuMemoryShare] Shutdown complete");
            }
            catch (Exception e)
            {
                Debug.LogError($"[GpuMemoryShare] Shutdown error: {e.Message}");
            }
        }

        /// <summary>
        /// Check if the system is initialized.
        /// </summary>
        public static bool IsInitialized => _initialized;

        /// <summary>
        /// Get the active backend.
        /// </summary>
        public static GpuShareBackend Backend
        {
            get
            {
                if (!_initialized) return GpuShareBackend.Auto;
                return GpuMemoryShare_GetBackend();
            }
        }

        /// <summary>
        /// Register a texture for sharing.
        /// </summary>
        /// <param name="texture">Unity texture to share</param>
        /// <param name="name">Name identifier</param>
        /// <param name="sharedTexture">Output: shared texture handle</param>
        /// <returns>True if registration succeeded</returns>
        public static bool RegisterTexture(RenderTexture texture, string name, out GpuShareTexture sharedTexture)
        {
            sharedTexture = default;

            if (!_initialized)
            {
                Debug.LogError("[GpuMemoryShare] Not initialized");
                return false;
            }

            if (texture == null)
            {
                Debug.LogError("[GpuMemoryShare] Texture is null");
                return false;
            }

            var format = GpuShareFormat.RGBA8;
            switch (texture.format)
            {
                case RenderTextureFormat.ARGB32:
                case RenderTextureFormat.BGRA32:
                    format = GpuShareFormat.BGRA8;
                    break;
                case RenderTextureFormat.RFloat:
                    format = GpuShareFormat.R32F;
                    break;
            }

            var namePtr = Marshal.StringToHGlobalAnsi(name);
            try
            {
                var info = new GpuShareTextureInfo
                {
                    NativeTexturePtr = texture.GetNativeTexturePtr(),
                    Width = texture.width,
                    Height = texture.height,
                    Format = format,
                    Name = namePtr,
                };

                var result = GpuMemoryShare_RegisterTexture(ref info, out sharedTexture);
                if (result != GpuShareError.Success)
                {
                    Debug.LogError($"[GpuMemoryShare] Failed to register texture: {result}");
                    return false;
                }

                Debug.Log($"[GpuMemoryShare] Registered texture '{name}' ({texture.width}x{texture.height})");
                return true;
            }
            finally
            {
                Marshal.FreeHGlobal(namePtr);
            }
        }

        /// <summary>
        /// Unregister a shared texture.
        /// </summary>
        public static void UnregisterTexture(ref GpuShareTexture texture)
        {
            if (!_initialized || texture.Handle == 0)
            {
                return;
            }

            GpuMemoryShare_UnregisterTexture(ref texture);
        }

        /// <summary>
        /// Notify that a frame is ready for consumption.
        /// Call this after Unity finishes rendering.
        /// </summary>
        public static bool NotifyFrameReady(ulong frameId, double timestamp, bool hasColor, bool hasDepth)
        {
            if (!_initialized)
            {
                return false;
            }

            var frameInfo = new GpuShareFrameInfo
            {
                FrameId = frameId,
                Timestamp = timestamp,
                HasColor = hasColor,
                HasDepth = hasDepth,
            };

            var result = GpuMemoryShare_NotifyFrameReady(ref frameInfo);
            return result == GpuShareError.Success;
        }

        #endregion
    }
}
