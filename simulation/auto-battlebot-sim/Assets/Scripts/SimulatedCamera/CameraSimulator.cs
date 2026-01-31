// Auto-Battlebot Simulation System
// CameraSimulator - RGB camera capture for TCP transfer (SIM-006)
//
// Renders to a RenderTexture that is read back via AsyncGPUReadback
// and sent to the C++ application over TCP. Configurable to match
// ZED 2i camera characteristics.

using System;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.Rendering;
using AutoBattlebot.Core;
using Debug = UnityEngine.Debug;

namespace AutoBattlebot.SimulatedCamera
{
    /// <summary>
    /// Resolution presets matching ZED 2i supported modes.
    /// </summary>
    public enum CameraResolutionPreset
    {
        /// <summary>HD720 - 1280x720 @ up to 60fps</summary>
        HD720,
        /// <summary>HD1080 - 1920x1080 @ up to 30fps</summary>
        HD1080,
        /// <summary>VGA - 640x360 @ up to 100fps</summary>
        VGA,
        /// <summary>Custom resolution</summary>
        Custom
    }

    /// <summary>
    /// Simulates an RGB camera that renders to a RenderTexture for AsyncGPUReadback.
    /// 
    /// This component:
    /// - Creates a RenderTexture compatible with AsyncGPUReadback (ARGB32, no MSAA)
    /// - Configures the camera FOV based on intrinsic parameters
    /// - Supports resolution presets matching ZED 2i modes
    /// - Provides performance profiling for optimization
    /// - Integrates with CommunicationBridge for TCP transfer
    /// 
    /// Usage:
    /// 1. Attach to a GameObject with a Camera component
    /// 2. Select a resolution preset or use custom resolution
    /// 3. CommunicationBridge will automatically use this texture for AsyncGPUReadback
    /// </summary>
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(CameraIntrinsicsProvider))]
    [ExecuteInEditMode]
    public class CameraSimulator : MonoBehaviour, IInitializable
    {
        #region Serialized Fields

        [Header("Resolution")]
        [SerializeField]
        [Tooltip("Resolution preset matching ZED 2i modes")]
        private CameraResolutionPreset _resolutionPreset = CameraResolutionPreset.HD720;

        [Header("Options")]
        [SerializeField]
        [Tooltip("Enable CPU readback for debugging (slower, use AsyncGPUReadback in production)")]
        private bool _enableSyncCpuReadback = false;

        [SerializeField]
        [Tooltip("Auto-create RenderTexture on Start")]
        private bool _autoCreate = true;

        [Header("Performance Profiling")]
        [SerializeField]
        [Tooltip("Enable performance profiling")]
        private bool _enableProfiling = false;

        [SerializeField]
        [Tooltip("Log profiling stats every N frames (0 to disable)")]
        private int _profilingLogInterval = 300;

        #endregion

        #region Private Fields

        private Camera _camera;
        private CameraIntrinsicsProvider _intrinsicsProvider;
        private RenderTexture _rgbTexture;
        private Texture2D _cpuTexture;
        private bool _isInitialized = false;

        // Profiling
        private Stopwatch _profilerStopwatch = new Stopwatch();
        private long _totalRenderTimeUs = 0;
        private long _totalReadbackTimeUs = 0;
        private int _frameCount = 0;
        private int _framesSinceLastLog = 0;

        #endregion

        #region Properties

        /// <summary>
        /// The RGB RenderTexture for AsyncGPUReadback.
        /// </summary>
        public RenderTexture RgbTexture => _rgbTexture;

        /// <summary>
        /// The attached Camera component.
        /// </summary>
        public Camera Camera => _camera;

        /// <summary>
        /// The attached CameraIntrinsicsProvider.
        /// </summary>
        public CameraIntrinsicsProvider IntrinsicsProvider => _intrinsicsProvider;

        /// <summary>
        /// Render width in pixels.
        /// </summary>
        public int Width => _intrinsicsProvider?.Width ?? GetPresetWidth(_resolutionPreset);

        /// <summary>
        /// Render height in pixels.
        /// </summary>
        public int Height => _intrinsicsProvider?.Height ?? GetPresetHeight(_resolutionPreset);

        /// <summary>
        /// Whether the camera system is initialized.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Whether the RenderTexture has been created.
        /// </summary>
        public bool HasTexture => _rgbTexture != null && _rgbTexture.IsCreated();

        /// <summary>
        /// Native texture pointer (for debugging/logging).
        /// </summary>
        public IntPtr NativeTexturePtr => HasTexture ? _rgbTexture.GetNativeTexturePtr() : IntPtr.Zero;

        /// <summary>
        /// Current resolution preset.
        /// </summary>
        public CameraResolutionPreset ResolutionPreset => _resolutionPreset;

        /// <summary>
        /// Average render time in microseconds.
        /// </summary>
        public double AverageRenderTimeUs => _frameCount > 0 ? (double)_totalRenderTimeUs / _frameCount : 0;

        /// <summary>
        /// Average readback time in microseconds.
        /// </summary>
        public double AverageReadbackTimeUs => _frameCount > 0 ? (double)_totalReadbackTimeUs / _frameCount : 0;

        #endregion

        #region IInitializable Implementation

        /// <summary>
        /// Initialization phase - runs during Init phase.
        /// </summary>
        public InitializationPhase Phase => InitializationPhase.Init;

        /// <summary>
        /// Initialize the camera system.
        /// </summary>
        void IInitializable.Initialize()
        {
            Initialize();
        }

        /// <summary>
        /// Shutdown the camera system.
        /// </summary>
        public void Shutdown()
        {
            Cleanup();
        }

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            _camera = GetComponent<Camera>();
            _intrinsicsProvider = GetComponent<CameraIntrinsicsProvider>();

            // Register with SimulationManager if it exists
            if (SimulationManager.Exists)
            {
                SimulationManager.Instance.RegisterInitializable(this);
            }
        }

        private void Start()
        {
            if (_autoCreate && Application.isPlaying && !_isInitialized)
            {
                Initialize();
            }
        }

        private void OnDestroy()
        {
            Cleanup();
        }

        private void OnValidate()
        {
            // Update camera settings in editor
            if (_camera == null)
            {
                _camera = GetComponent<Camera>();
            }

            if (_intrinsicsProvider == null)
            {
                _intrinsicsProvider = GetComponent<CameraIntrinsicsProvider>();
            }

            // Apply resolution preset to intrinsics provider
            if (_intrinsicsProvider != null && _resolutionPreset != CameraResolutionPreset.Custom)
            {
                ApplyResolutionPreset();
            }

            if (_camera != null)
            {
                ApplyCameraSettings();
            }
        }

        private void LateUpdate()
        {
            if (!_isInitialized || !_enableProfiling)
            {
                return;
            }

            _framesSinceLastLog++;
            if (_profilingLogInterval > 0 && _framesSinceLastLog >= _profilingLogInterval)
            {
                LogProfilingStats();
                _framesSinceLastLog = 0;
            }
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Initialize the camera system and create the RenderTexture.
        /// </summary>
        public bool Initialize()
        {
            if (_isInitialized)
            {
                Debug.LogWarning("[CameraSimulator] Already initialized");
                return true;
            }

            if (_camera == null)
            {
                _camera = GetComponent<Camera>();
            }

            if (_camera == null)
            {
                Debug.LogError("[CameraSimulator] No Camera component found");
                return false;
            }

            if (_intrinsicsProvider == null)
            {
                _intrinsicsProvider = GetComponent<CameraIntrinsicsProvider>();
            }

            // Apply resolution preset if not custom
            if (_resolutionPreset != CameraResolutionPreset.Custom)
            {
                ApplyResolutionPreset();
            }

            // Create RenderTexture
            if (!CreateRenderTexture())
            {
                return false;
            }

            // Apply camera settings
            ApplyCameraSettings();

            _isInitialized = true;
            Debug.Log($"[CameraSimulator] Initialized: {Width}x{Height}, " +
                      $"Preset={_resolutionPreset}, " +
                      $"FOV={_camera.fieldOfView:F1}°, " +
                      $"Format={_rgbTexture.format}");

            return true;
        }

        /// <summary>
        /// Cleanup resources.
        /// </summary>
        public void Cleanup()
        {
            if (_cpuTexture != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(_cpuTexture);
                }
                else
                {
                    DestroyImmediate(_cpuTexture);
                }
                _cpuTexture = null;
            }

            if (_rgbTexture != null)
            {
                _rgbTexture.Release();
                if (Application.isPlaying)
                {
                    Destroy(_rgbTexture);
                }
                else
                {
                    DestroyImmediate(_rgbTexture);
                }
                _rgbTexture = null;
            }

            if (_camera != null)
            {
                _camera.targetTexture = null;
            }

            _isInitialized = false;
        }

        /// <summary>
        /// Change resolution preset at runtime.
        /// </summary>
        public void SetResolutionPreset(CameraResolutionPreset preset)
        {
            if (_resolutionPreset == preset)
            {
                return;
            }

            _resolutionPreset = preset;
            
            if (_isInitialized)
            {
                ApplyResolutionPreset();
                RecreateRenderTexture();
            }
        }

        /// <summary>
        /// Read pixels to CPU synchronously (for debugging or data generation).
        /// Warning: This blocks and is slow. Use AsyncGPUReadback for production.
        /// </summary>
        /// <returns>CPU Texture2D or null if readback is disabled</returns>
        public Texture2D ReadToCpuSync()
        {
            if (!_enableSyncCpuReadback)
            {
                Debug.LogWarning("[CameraSimulator] Sync CPU readback is disabled. Enable in Inspector or use AsyncGPUReadback.");
                return null;
            }

            if (!HasTexture)
            {
                return null;
            }

            _profilerStopwatch.Restart();

            // Create CPU texture if needed
            if (_cpuTexture == null || _cpuTexture.width != Width || _cpuTexture.height != Height)
            {
                if (_cpuTexture != null)
                {
                    Destroy(_cpuTexture);
                }
                _cpuTexture = new Texture2D(Width, Height, TextureFormat.RGBA32, false);
            }

            // Read from GPU to CPU (blocking)
            var prevActive = RenderTexture.active;
            RenderTexture.active = _rgbTexture;
            _cpuTexture.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
            _cpuTexture.Apply();
            RenderTexture.active = prevActive;

            _profilerStopwatch.Stop();
            if (_enableProfiling)
            {
                _totalReadbackTimeUs += _profilerStopwatch.ElapsedTicks * 1000000 / Stopwatch.Frequency;
                _frameCount++;
            }

            return _cpuTexture;
        }

        /// <summary>
        /// Get performance report string.
        /// </summary>
        public string GetPerformanceReport()
        {
            return $"[CameraSimulator] Performance:\n" +
                   $"  Resolution: {Width}x{Height}\n" +
                   $"  Preset: {_resolutionPreset}\n" +
                   $"  Frames: {_frameCount}\n" +
                   $"  Avg Readback: {AverageReadbackTimeUs:F1}µs";
        }

        /// <summary>
        /// Reset profiling statistics.
        /// </summary>
        public void ResetProfilingStats()
        {
            _totalRenderTimeUs = 0;
            _totalReadbackTimeUs = 0;
            _frameCount = 0;
        }

        #endregion

        #region Private Methods

        private void ApplyResolutionPreset()
        {
            if (_intrinsicsProvider == null)
            {
                return;
            }

            int width = GetPresetWidth(_resolutionPreset);
            int height = GetPresetHeight(_resolutionPreset);

            // Get ZED 2i intrinsics for this resolution
            var intrinsics = GetZed2iIntrinsics(_resolutionPreset);
            _intrinsicsProvider.SetIntrinsics(intrinsics);
        }

        private bool CreateRenderTexture()
        {
            // Clean up existing texture
            if (_rgbTexture != null)
            {
                _rgbTexture.Release();
                if (Application.isPlaying)
                {
                    Destroy(_rgbTexture);
                }
                else
                {
                    DestroyImmediate(_rgbTexture);
                }
            }

            // Create AsyncGPUReadback-compatible RenderTexture
            // - ARGB32 format (compatible with all platforms and AsyncGPUReadback)
            // - No MSAA (AsyncGPUReadback doesn't support MSAA textures)
            // - No mipmaps (not needed for readback)
            // - Depth buffer bits = 0 (separate depth texture via LinearDepthCapturePass)
            _rgbTexture = new RenderTexture(Width, Height, 0, RenderTextureFormat.ARGB32)
            {
                antiAliasing = 1,
                useMipMap = false,
                autoGenerateMips = false,
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                name = $"CameraSimulator_RGB_{Width}x{Height}"
            };

            if (!_rgbTexture.Create())
            {
                Debug.LogError("[CameraSimulator] Failed to create RenderTexture");
                return false;
            }

            // Set as camera target
            _camera.targetTexture = _rgbTexture;

            return true;
        }

        private void RecreateRenderTexture()
        {
            if (_camera != null)
            {
                _camera.targetTexture = null;
            }

            if (!CreateRenderTexture())
            {
                Debug.LogError("[CameraSimulator] Failed to recreate RenderTexture");
                return;
            }

            ApplyCameraSettings();

            Debug.Log($"[CameraSimulator] Recreated texture: {Width}x{Height}");
        }

        private void ApplyCameraSettings()
        {
            if (_camera == null || _intrinsicsProvider == null)
            {
                return;
            }

            // Calculate vertical FOV from focal length
            // FOV = 2 * atan(height / (2 * fy))
            double fovRadians = 2.0 * Math.Atan(Height / (2.0 * _intrinsicsProvider.Fy));
            float fovDegrees = (float)(fovRadians * 180.0 / Math.PI);

            _camera.fieldOfView = fovDegrees;
            _camera.nearClipPlane = _intrinsicsProvider.NearClip;
            _camera.farClipPlane = _intrinsicsProvider.FarClip;

            // Apply custom projection matrix for off-center principal point
            if (Math.Abs(_intrinsicsProvider.Cx - Width / 2.0) > 1.0 || 
                Math.Abs(_intrinsicsProvider.Cy - Height / 2.0) > 1.0)
            {
                ApplyCustomProjectionMatrix();
            }
        }

        private void ApplyCustomProjectionMatrix()
        {
            // Create asymmetric projection matrix for off-center principal point
            // This is important for accurate camera simulation
            float near = _intrinsicsProvider.NearClip;
            float far = _intrinsicsProvider.FarClip;

            // Calculate frustum boundaries at near plane
            float left = (float)(-_intrinsicsProvider.Cx * near / _intrinsicsProvider.Fx);
            float right = (float)((Width - _intrinsicsProvider.Cx) * near / _intrinsicsProvider.Fx);
            float bottom = (float)(-(Height - _intrinsicsProvider.Cy) * near / _intrinsicsProvider.Fy);
            float top = (float)(_intrinsicsProvider.Cy * near / _intrinsicsProvider.Fy);

            // Create OpenGL-style projection matrix
            Matrix4x4 proj = Matrix4x4.zero;
            proj[0, 0] = 2.0f * near / (right - left);
            proj[1, 1] = 2.0f * near / (top - bottom);
            proj[0, 2] = (right + left) / (right - left);
            proj[1, 2] = (top + bottom) / (top - bottom);
            proj[2, 2] = -(far + near) / (far - near);
            proj[2, 3] = -2.0f * far * near / (far - near);
            proj[3, 2] = -1.0f;

            _camera.projectionMatrix = proj;
        }

        private void LogProfilingStats()
        {
            Debug.Log(GetPerformanceReport());
        }

        #endregion

        #region Static Helper Methods

        /// <summary>
        /// Get width for a resolution preset.
        /// </summary>
        public static int GetPresetWidth(CameraResolutionPreset preset)
        {
            return preset switch
            {
                CameraResolutionPreset.HD720 => 1280,
                CameraResolutionPreset.HD1080 => 1920,
                CameraResolutionPreset.VGA => 640,
                _ => 1280
            };
        }

        /// <summary>
        /// Get height for a resolution preset.
        /// </summary>
        public static int GetPresetHeight(CameraResolutionPreset preset)
        {
            return preset switch
            {
                CameraResolutionPreset.HD720 => 720,
                CameraResolutionPreset.HD1080 => 1080,
                CameraResolutionPreset.VGA => 360,
                _ => 720
            };
        }

        /// <summary>
        /// Get ZED 2i camera intrinsics for a resolution preset.
        /// Values from ZED SDK calibration.
        /// </summary>
        public static CameraIntrinsics GetZed2iIntrinsics(CameraResolutionPreset preset)
        {
            // ZED 2i intrinsics from SDK (left camera, rectified)
            // Note: These are typical values; actual values vary per camera
            return preset switch
            {
                CameraResolutionPreset.HD1080 => CameraIntrinsics.Create(
                    width: 1920,
                    height: 1080,
                    fx: 1061.4892578125,
                    fy: 1061.4892578125,
                    cx: 971.2513427734375,
                    cy: 561.7954711914062,
                    k1: 0, k2: 0, p1: 0, p2: 0, k3: 0  // Rectified = no distortion
                ),
                CameraResolutionPreset.HD720 => CameraIntrinsics.Create(
                    width: 1280,
                    height: 720,
                    fx: 707.6595458984375,  // Scaled from 1080p
                    fy: 707.6595458984375,
                    cx: 647.5008544921875,
                    cy: 374.5303039550781,
                    k1: 0, k2: 0, p1: 0, p2: 0, k3: 0
                ),
                CameraResolutionPreset.VGA => CameraIntrinsics.Create(
                    width: 640,
                    height: 360,
                    fx: 353.82977294921875,  // Scaled from 1080p
                    fy: 353.82977294921875,
                    cx: 323.75042724609375,
                    cy: 187.26515197753906,
                    k1: 0, k2: 0, p1: 0, p2: 0, k3: 0
                ),
                _ => CameraIntrinsics.CreateDefault(1280, 720)
            };
        }

        #endregion

        #region Editor Support

#if UNITY_EDITOR
        [ContextMenu("Reinitialize")]
        private void EditorReinitialize()
        {
            Cleanup();
            Initialize();
        }

        [ContextMenu("Log Texture Info")]
        private void EditorLogTextureInfo()
        {
            if (HasTexture)
            {
                Debug.Log($"[CameraSimulator] Texture: {_rgbTexture.width}x{_rgbTexture.height}, " +
                          $"Format={_rgbTexture.format}, " +
                          $"Preset={_resolutionPreset}");
            }
            else
            {
                Debug.Log("[CameraSimulator] No texture created");
            }
        }

        [ContextMenu("Apply HD720 Preset")]
        private void EditorApplyHD720()
        {
            SetResolutionPreset(CameraResolutionPreset.HD720);
        }

        [ContextMenu("Apply HD1080 Preset")]
        private void EditorApplyHD1080()
        {
            SetResolutionPreset(CameraResolutionPreset.HD1080);
        }

        [ContextMenu("Apply VGA Preset")]
        private void EditorApplyVGA()
        {
            SetResolutionPreset(CameraResolutionPreset.VGA);
        }

        [ContextMenu("Log Performance Report")]
        private void EditorLogPerformance()
        {
            Debug.Log(GetPerformanceReport());
        }
#endif

        #endregion
    }
}
