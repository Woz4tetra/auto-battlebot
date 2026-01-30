// Auto-Battlebot Simulation System
// CameraSimulator - RGB camera capture for CUDA Interop (SIM-006)
//
// Renders to a RenderTexture that can be registered with CUDA for
// zero-copy GPU texture sharing. Configurable to match ZED 2i camera
// characteristics.

using System;
using UnityEngine;
using UnityEngine.Rendering;

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
    [ExecuteInEditMode]
    public class CameraSimulator : MonoBehaviour
    {
        #region Serialized Fields

        [Header("Resolution")]
        [SerializeField]
        [Tooltip("Render texture width in pixels")]
        private int _width = 1280;

        [SerializeField]
        [Tooltip("Render texture height in pixels")]
        private int _height = 720;

        [Header("Camera Intrinsics")]
        [SerializeField]
        [Tooltip("Focal length X in pixels (affects FOV calculation)")]
        private double _fx = 707.66;

        [SerializeField]
        [Tooltip("Focal length Y in pixels")]
        private double _fy = 707.66;

        [SerializeField]
        [Tooltip("Principal point X (typically width/2)")]
        private double _cx = 640.0;

        [SerializeField]
        [Tooltip("Principal point Y (typically height/2)")]
        private double _cy = 360.0;

        [Header("Depth Range")]
        [SerializeField]
        [Tooltip("Near clip plane in meters")]
        private float _nearClip = 0.3f;

        [SerializeField]
        [Tooltip("Far clip plane in meters")]
        private float _farClip = 20.0f;

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
        public int Width => _width;

        /// <summary>
        /// Render height in pixels.
        /// </summary>
        public int Height => _height;

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
            // Clamp values
            _width = Mathf.Max(64, _width);
            _height = Mathf.Max(64, _height);
            _fx = Math.Max(1.0, _fx);
            _fy = Math.Max(1.0, _fy);
            _nearClip = Mathf.Max(0.01f, _nearClip);
            _farClip = Mathf.Max(_nearClip + 0.1f, _farClip);

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
            Debug.Log($"[CameraSimulator] Initialized: {_width}x{_height}, " +
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
        /// Recreate the RenderTexture with new dimensions.
        /// </summary>
        public bool Resize(int width, int height)
        {
            _width = width;
            _height = height;

            Cleanup();
            return Initialize();
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
            if (_cpuTexture == null || _cpuTexture.width != _width || _cpuTexture.height != _height)
            {
                if (_cpuTexture != null)
                {
                    Destroy(_cpuTexture);
                }
                _cpuTexture = new Texture2D(_width, _height, TextureFormat.RGBA32, false);
            }

            // Read from GPU to CPU
            var prevActive = RenderTexture.active;
            RenderTexture.active = _rgbTexture;
            _cpuTexture.ReadPixels(new Rect(0, 0, _width, _height), 0, 0);
            _cpuTexture.Apply();
            RenderTexture.active = prevActive;

            return _cpuTexture;
        }

        /// <summary>
        /// Set camera intrinsics programmatically.
        /// </summary>
        public void SetIntrinsics(double fx, double fy, double cx, double cy)
        {
            _fx = fx;
            _fy = fy;
            _cx = cx;
            _cy = cy;

            ApplyCameraSettings();
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
            _rgbTexture = new RenderTexture(_width, _height, 0, RenderTextureFormat.ARGB32)
            {
                antiAliasing = 1,
                useMipMap = false,
                autoGenerateMips = false,
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                name = $"CameraSimulator_RGB_{_width}x{_height}"
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
            if (_camera == null)
            {
                return;
            }

            // Calculate vertical FOV from focal length
            // FOV = 2 * atan(height / (2 * fy))
            double fovRadians = 2.0 * Math.Atan(_height / (2.0 * _fy));
            float fovDegrees = (float)(fovRadians * 180.0 / Math.PI);

            _camera.fieldOfView = fovDegrees;
            _camera.nearClipPlane = _nearClip;
            _camera.farClipPlane = _farClip;

            // Note: Unity doesn't natively support off-center principal points.
            // For accurate simulation with cx != width/2 or cy != height/2,
            // a custom projection matrix would be needed.
            if (Math.Abs(_cx - _width / 2.0) > 1.0 || Math.Abs(_cy - _height / 2.0) > 1.0)
            {
                // Apply custom projection matrix for off-center principal point
                ApplyCustomProjectionMatrix();
            }
        }

        private void ApplyCustomProjectionMatrix()
        {
            // Create asymmetric projection matrix for off-center principal point
            // This is important for accurate camera simulation
            float near = _nearClip;
            float far = _farClip;

            // Calculate frustum boundaries at near plane
            float left = (float)(-_cx * near / _fx);
            float right = (float)((_width - _cx) * near / _fx);
            float bottom = (float)(-(_height - _cy) * near / _fy);
            float top = (float)(_cy * near / _fy);

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

        [ContextMenu("Apply ZED 2i 720p Preset")]
        private void EditorApplyZed720p()
        {
            _width = 1280;
            _height = 720;
            _fx = 707.66;
            _fy = 707.66;
            _cx = 647.50;
            _cy = 374.53;
            _nearClip = 0.3f;
            _farClip = 20.0f;

            if (_camera == null)
            {
                _camera = GetComponent<UnityEngine.Camera>();
            }
            ApplyCameraSettings();

            if (_isInitialized)
            {
                Cleanup();
                Initialize();
            }
        }

        [ContextMenu("Apply ZED 2i 1080p Preset")]
        private void EditorApplyZed1080p()
        {
            _width = 1920;
            _height = 1080;
            _fx = 1061.49;
            _fy = 1061.49;
            _cx = 971.25;
            _cy = 561.80;
            _nearClip = 0.3f;
            _farClip = 20.0f;

            if (_camera == null)
            {
                _camera = GetComponent<UnityEngine.Camera>();
            }
            ApplyCameraSettings();

            if (_isInitialized)
            {
                Cleanup();
                Initialize();
            }
        }
#endif

        #endregion
    }
}
