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
    /// Simulates an RGB camera that renders to a RenderTexture for AsyncGPUReadback.
    /// 
    /// This component:
    /// - Creates a RenderTexture compatible with AsyncGPUReadback (ARGB32, no MSAA)
    /// - Configures the camera FOV based on intrinsic parameters
    /// - Provides performance profiling for optimization
    /// - Integrates with CommunicationBridge for TCP transfer
    /// 
    /// Usage:
    /// 1. Attach to a GameObject with a Camera component
    /// 2. Fill out the CameraIntrinsicsProvider component
    /// 3. CommunicationBridge will automatically use this texture for AsyncGPUReadback
    /// </summary>
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(CameraIntrinsicsProvider))]
    [ExecuteInEditMode]
    public class CameraSimulator : MonoBehaviour, IInitializable
    {
        #region Serialized Fields

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
        private Matrix4x4 tfWorldFromCamerastart;

        // Profiling
        private Stopwatch _profilerStopwatch = new Stopwatch();
        private long _totalRenderTimeUs = 0;
        private long _totalReadbackTimeUs = 0;
        private int _frameCount = 0;
        private int _framesSinceLastLog = 0;

        // Projection matrix verification
        private Matrix4x4 _expectedProjectionMatrix;
        private bool _projectionMatrixWarned = false;

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
        public int Width => _intrinsicsProvider.Width;

        /// <summary>
        /// Render height in pixels.
        /// </summary>
        public int Height => _intrinsicsProvider.Height;

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

            if (_camera != null)
            {
                ApplyCameraSettings();
            }
        }

        private void LateUpdate()
        {
            if (!_isInitialized)
            {
                return;
            }

            // Verify projection matrix hasn't been overridden by HDRP
            VerifyProjectionMatrix();

            if (!_enableProfiling)
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

            // Create RenderTexture
            if (!CreateRenderTexture())
            {
                return false;
            }

            // Apply camera settings
            ApplyCameraSettings();

            _isInitialized = true;
            Debug.Log($"[CameraSimulator] Initialized: {Width}x{Height}, " +
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

        public Matrix4x4 GetCameraPose()
        {
            Matrix4x4 tfWorldFromCamera = _camera.worldToCameraMatrix;
            return tfWorldFromCamerastart.inverse * tfWorldFromCamera;
        }

        #endregion

        #region Private Methods

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
            // Use GraphicsFormat for explicit format control in HDRP
            // - R8G8B8A8_SRGB: Standard sRGB format, compatible with AsyncGPUReadback
            // - No MSAA (AsyncGPUReadback doesn't support MSAA textures)
            // - No mipmaps (not needed for readback)
            // - Depth buffer bits = 0 (separate depth texture via LinearDepthCapturePass)
            var desc = new RenderTextureDescriptor(Width, Height)
            {
                graphicsFormat = UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB,
                depthBufferBits = 0,
                msaaSamples = 1,
                useMipMap = false,
                autoGenerateMips = false,
                enableRandomWrite = false,
                dimension = UnityEngine.Rendering.TextureDimension.Tex2D,
                volumeDepth = 1
            };

            _rgbTexture = new RenderTexture(desc)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                name = $"CameraSimulator_RGB_{Width}x{Height}"
            };

            if (!_rgbTexture.Create())
            {
                Debug.LogError("[CameraSimulator] Failed to create RenderTexture");
                return false;
            }

            Debug.Log($"[CameraSimulator] Created RenderTexture: {Width}x{Height}, " +
                      $"Format={_rgbTexture.graphicsFormat}, " +
                      $"ColorFormat={_rgbTexture.format}");

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

            _camera.nearClipPlane = _intrinsicsProvider.NearClip;
            _camera.farClipPlane = _intrinsicsProvider.FarClip;

            // ALWAYS apply custom projection matrix to ensure exact intrinsic matching
            // Unity's FOV-based projection doesn't account for:
            // - Off-center principal points
            // - Non-square pixels (fx != fy)
            // - Exact focal length values
            ApplyCustomProjectionMatrix();
        }

        private void ApplyCustomProjectionMatrix()
        {
            // Create projection matrix that exactly matches the camera intrinsics
            // This ensures depth-to-point-cloud conversion will align with the RGB image
            float near = _intrinsicsProvider.NearClip;
            float far = _intrinsicsProvider.FarClip;
            float fx = (float)_intrinsicsProvider.Fx;
            float fy = (float)_intrinsicsProvider.Fy;
            float cx = (float)_intrinsicsProvider.Cx;
            float cy = (float)_intrinsicsProvider.Cy;
            float w = Width;
            float h = Height;

            // Calculate frustum boundaries at near plane
            // For pixel (u, v), the 3D point is at ((u - cx) * Z / fx, (v - cy) * Z / fy, Z)
            // At the edges of the image:
            //   u = 0      -> X = -cx * Z / fx
            //   u = width  -> X = (width - cx) * Z / fx
            //   v = 0      -> Y = -cy * Z / fy  (top of image)
            //   v = height -> Y = (height - cy) * Z / fy (bottom of image)
            //
            // Unity's camera has Y-up, so we need to flip Y:
            //   top (v=0) in image -> top in Unity (positive Y)
            //   bottom (v=height) in image -> bottom in Unity (negative Y)
            float left = -cx * near / fx;
            float right = (w - cx) * near / fx;
            float bottom = -(h - cy) * near / fy;  // v=height maps to negative Y
            float top = cy * near / fy;            // v=0 maps to positive Y

            // Create OpenGL-style projection matrix (Unity uses OpenGL conventions)
            // This matrix transforms from camera space to clip space
            Matrix4x4 proj = Matrix4x4.zero;
            proj[0, 0] = 2.0f * near / (right - left);           // = 2 * fx / width
            proj[1, 1] = 2.0f * near / (top - bottom);           // = 2 * fy / height
            proj[0, 2] = (right + left) / (right - left);        // = (width - 2*cx) / width
            proj[1, 2] = (top + bottom) / (top - bottom);        // = (2*cy - height) / height
            proj[2, 2] = -(far + near) / (far - near);
            proj[2, 3] = -2.0f * far * near / (far - near);
            proj[3, 2] = -1.0f;

            _camera.projectionMatrix = proj;
            _expectedProjectionMatrix = proj;
            _projectionMatrixWarned = false;

            // Log projection matrix details for debugging
            Debug.Log($"[CameraSimulator] Applied projection matrix from intrinsics:\n" +
                      $"  Intrinsics: fx={fx:F2}, fy={fy:F2}, cx={cx:F2}, cy={cy:F2}\n" +
                      $"  Resolution: {w}x{h}\n" +
                      $"  Frustum: left={left:F4}, right={right:F4}, bottom={bottom:F4}, top={top:F4}\n" +
                      $"  proj[0,0]={proj[0,0]:F4} (expected: {2*fx/w:F4})\n" +
                      $"  proj[1,1]={proj[1,1]:F4} (expected: {2*fy/h:F4})\n" +
                      $"  proj[0,2]={proj[0,2]:F4} (expected: {(w - 2*cx)/w:F4})\n" +
                      $"  proj[1,2]={proj[1,2]:F4} (expected: {(2*cy - h)/h:F4})");
        }

        /// <summary>
        /// Verify the projection matrix hasn't been overridden by HDRP or other systems.
        /// Call this in LateUpdate or after rendering to detect issues.
        /// </summary>
        private void VerifyProjectionMatrix()
        {
            if (_camera == null || _expectedProjectionMatrix == Matrix4x4.zero)
            {
                return;
            }

            var actual = _camera.projectionMatrix;
            bool mismatch = false;
            
            // Check key projection matrix elements
            // Allow small tolerance for floating point differences
            const float tolerance = 0.001f;
            if (Math.Abs(actual[0, 0] - _expectedProjectionMatrix[0, 0]) > tolerance ||
                Math.Abs(actual[1, 1] - _expectedProjectionMatrix[1, 1]) > tolerance ||
                Math.Abs(actual[0, 2] - _expectedProjectionMatrix[0, 2]) > tolerance ||
                Math.Abs(actual[1, 2] - _expectedProjectionMatrix[1, 2]) > tolerance)
            {
                mismatch = true;
            }

            if (mismatch && !_projectionMatrixWarned)
            {
                _projectionMatrixWarned = true;
                Debug.LogError($"[CameraSimulator] PROJECTION MATRIX MISMATCH DETECTED!\n" +
                              $"The camera's projection matrix was overridden (likely by HDRP).\n" +
                              $"This will cause point cloud misalignment.\n" +
                              $"Expected:\n" +
                              $"  [0,0]={_expectedProjectionMatrix[0,0]:F4}, [1,1]={_expectedProjectionMatrix[1,1]:F4}\n" +
                              $"  [0,2]={_expectedProjectionMatrix[0,2]:F4}, [1,2]={_expectedProjectionMatrix[1,2]:F4}\n" +
                              $"Actual:\n" +
                              $"  [0,0]={actual[0,0]:F4}, [1,1]={actual[1,1]:F4}\n" +
                              $"  [0,2]={actual[0,2]:F4}, [1,2]={actual[1,2]:F4}");
                
                // Re-apply the projection matrix
                _camera.projectionMatrix = _expectedProjectionMatrix;
            }
        }

        private void LogProfilingStats()
        {
            Debug.Log(GetPerformanceReport());
        }

        private void InitializeCameraPose()
        {
            tfWorldFromCamerastart = _camera.worldToCameraMatrix;
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
                          $"Format={_rgbTexture.format}, ");
            }
            else
            {
                Debug.Log("[CameraSimulator] No texture created");
            }
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
