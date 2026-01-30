// Auto-Battlebot Simulation System
// CameraSimulator - RGB camera capture for CUDA Interop (SIM-006)
//
// Renders to a RenderTexture that can be registered with CUDA for
// zero-copy GPU texture sharing. Configurable to match ZED 2i camera
// characteristics.

using System;
using UnityEngine;
using AutoBattlebot.Core;

namespace AutoBattlebot.Camera
{
    /// <summary>
    /// Simulates an RGB camera that renders to a RenderTexture for CUDA Interop.
    /// 
    /// This component:
    /// - Creates a RenderTexture compatible with CUDA (ARGB32, no MSAA)
    /// - Configures the camera FOV based on intrinsic parameters
    /// - Exposes the native texture pointer for CUDA registration
    /// - Optionally provides CPU readback for debugging/data generation
    /// 
    /// Usage:
    /// 1. Attach to a GameObject with a Camera component
    /// 2. Configure resolution and intrinsics in Inspector
    /// 3. Access RgbTexture.GetNativeTexturePtr() for CUDA registration
    /// </summary>
    [RequireComponent(typeof(UnityEngine.Camera))]
    [RequireComponent(typeof(CameraIntrinsicsProvider))]
    [ExecuteInEditMode]
    public class CameraSimulator : MonoBehaviour
    {
        #region Serialized Fields

        [Header("Options")]
        [SerializeField]
        [Tooltip("Enable CPU readback for debugging (slower)")]
        private bool _enableCpuReadback = false;

        [SerializeField]
        [Tooltip("Auto-create RenderTexture on Start")]
        private bool _autoCreate = true;

        #endregion

        #region Private Fields

        private UnityEngine.Camera _camera;
        private CameraIntrinsicsProvider _intrinsics_provider;
        private RenderTexture _rgbTexture;
        private Texture2D _cpuTexture;
        private bool _isInitialized = false;

        #endregion

        #region Properties

        /// <summary>
        /// The RGB RenderTexture for CUDA registration.
        /// Use GetNativeTexturePtr() to get the OpenGL texture handle.
        /// </summary>
        public RenderTexture RgbTexture => _rgbTexture;

        /// <summary>
        /// The attached Camera component.
        /// </summary>
        public UnityEngine.Camera Camera => _camera;

        /// <summary>
        /// Render width in pixels.
        /// </summary>
        public int Width => _intrinsics_provider.Width;

        /// <summary>
        /// Render height in pixels.
        /// </summary>
        public int Height => _intrinsics_provider.Height;

        /// <summary>
        /// Whether the camera system is initialized.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Whether the RenderTexture has been created.
        /// </summary>
        public bool HasTexture => _rgbTexture != null && _rgbTexture.IsCreated();

        /// <summary>
        /// Native texture pointer for CUDA registration.
        /// </summary>
        public IntPtr NativeTexturePtr => HasTexture ? _rgbTexture.GetNativeTexturePtr() : IntPtr.Zero;

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            _camera = GetComponent<UnityEngine.Camera>();
            _intrinsics_provider = GetComponent<CameraIntrinsicsProvider>();
        }

        private void Start()
        {
            if (_autoCreate && Application.isPlaying)
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
                _camera = GetComponent<UnityEngine.Camera>();
            }

            if (_camera != null)
            {
                ApplyCameraSettings();
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
                _camera = GetComponent<UnityEngine.Camera>();
            }

            if (_camera == null)
            {
                Debug.LogError("[CameraSimulator] No Camera component found");
                return false;
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
                      $"NativePtr=0x{NativeTexturePtr.ToInt64():X}");

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
        /// Read pixels to CPU (for debugging or data generation).
        /// Only works if _enableCpuReadback is true.
        /// </summary>
        /// <returns>CPU Texture2D or null if readback is disabled</returns>
        public Texture2D ReadToCpu()
        {
            if (!_enableCpuReadback)
            {
                Debug.LogWarning("[CameraSimulator] CPU readback is disabled");
                return null;
            }

            if (!HasTexture)
            {
                return null;
            }

            // Create CPU texture if needed
            if (_cpuTexture == null || _cpuTexture.width != Width || _cpuTexture.height != Height)
            {
                if (_cpuTexture != null)
                {
                    Destroy(_cpuTexture);
                }
                _cpuTexture = new Texture2D(Width, Height, TextureFormat.RGBA32, false);
            }

            // Read from GPU to CPU
            var prevActive = RenderTexture.active;
            RenderTexture.active = _rgbTexture;
            _cpuTexture.ReadPixels(new Rect(0, 0, Width, Height), 0, 0);
            _cpuTexture.Apply();
            RenderTexture.active = prevActive;

            return _cpuTexture;
        }

        #endregion

        #region Private Methods

        private bool CreateRenderTexture()
        {
            // Create CUDA-compatible RenderTexture
            // - ARGB32 format (CUDA maps this to cudaChannelFormatKindUnsigned)
            // - No MSAA (CUDA doesn't support MSAA textures)
            // - No mipmaps (not needed for inference)
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

        private void ApplyCameraSettings()
        {
            if (_camera == null || _intrinsics_provider == null)
            {
                return;
            }

            // Calculate vertical FOV from focal length
            // FOV = 2 * atan(height / (2 * fy))
            double fovRadians = 2.0 * Math.Atan(Height / (2.0 * _intrinsics_provider.Fx));
            float fovDegrees = (float)(fovRadians * 180.0 / Math.PI);

            _camera.fieldOfView = fovDegrees;
            _camera.nearClipPlane = _intrinsics_provider.NearClip;
            _camera.farClipPlane = _intrinsics_provider.FarClip;

            // Note: Unity doesn't natively support off-center principal points.
            // For accurate simulation with cx != width/2 or cy != height/2,
            // a custom projection matrix would be needed.
            if (Math.Abs(_intrinsics_provider.Cx - Width / 2.0) > 1.0 || Math.Abs(_intrinsics_provider.Cy - Height / 2.0) > 1.0)
            {
                // Apply custom projection matrix for off-center principal point
                ApplyCustomProjectionMatrix();
            }
        }

        private void ApplyCustomProjectionMatrix()
        {
            // Create asymmetric projection matrix for off-center principal point
            // This is important for accurate camera simulation
            float near = _intrinsics_provider.NearClip;
            float far = _intrinsics_provider.FarClip;

            // Calculate frustum boundaries at near plane
            float left = (float)(-_intrinsics_provider.Cx * near / _intrinsics_provider.Fx);
            float right = (float)((Width - _intrinsics_provider.Cx) * near / _intrinsics_provider.Fx);
            float bottom = (float)(-(Height - _intrinsics_provider.Cy) * near / _intrinsics_provider.Fy);
            float top = (float)(_intrinsics_provider.Cy * near / _intrinsics_provider.Fy);

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
                          $"NativePtr=0x{NativeTexturePtr.ToInt64():X}");
            }
            else
            {
                Debug.Log("[CameraSimulator] No texture created");
            }
        }
#endif

        #endregion
    }
}
