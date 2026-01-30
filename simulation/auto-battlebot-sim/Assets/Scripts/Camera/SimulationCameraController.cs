// Auto-Battlebot Simulation System
// SimulationCameraController - Unified controller for RGB and Depth capture
//
// Coordinates CameraSimulator (RGB) and LinearDepthCapturePass (Depth)
// to provide synchronized RGB-D frames for CUDA Interop.

using System;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace AutoBattlebot.Camera
{
    /// <summary>
    /// Unified controller for simulation camera capture.
    /// 
    /// Coordinates:
    /// - CameraSimulator for RGB capture
    /// - LinearDepthCapturePass for depth capture
    /// - Provides synchronized access to both textures
    /// 
    /// Usage:
    /// 1. Attach to the same GameObject as the Camera
    /// 2. Assign the compute shader and (optionally) a Custom Pass Volume
    /// 3. Call Initialize() or rely on auto-initialization
    /// 4. Access RgbTexture and DepthTexture for CUDA registration
    /// </summary>
    [RequireComponent(typeof(CameraSimulator))]
    public class SimulationCameraController : MonoBehaviour
    {
        #region Serialized Fields

        [Header("Depth Capture")]
        [SerializeField]
        [Tooltip("Compute shader for depth linearization")]
        private ComputeShader _linearizeDepthShader;

        [SerializeField]
        [Tooltip("Custom Pass Volume (auto-created if null)")]
        private CustomPassVolume _customPassVolume;

        [Header("Configuration")]
        [SerializeField]
        [Tooltip("Auto-initialize on Start")]
        private bool _autoInitialize = true;

        [SerializeField]
        [Tooltip("Enable depth capture")]
        private bool _enableDepth = true;

        [SerializeField]
        [Tooltip("Value for invalid/sky depth (0 = no depth)")]
        private float _invalidDepthValue = 0.0f;

        #endregion

        #region Private Fields

        private CameraSimulator _cameraSimulator;
        private LinearDepthCapturePass _depthPass;
        private bool _isInitialized = false;
        private bool _ownsCustomPassVolume = false;

        #endregion

        #region Properties

        /// <summary>
        /// The CameraSimulator component for RGB capture.
        /// </summary>
        public CameraSimulator CameraSimulator => _cameraSimulator;

        /// <summary>
        /// The LinearDepthCapturePass for depth capture.
        /// </summary>
        public LinearDepthCapturePass DepthPass => _depthPass;

        /// <summary>
        /// The RGB RenderTexture.
        /// </summary>
        public RenderTexture RgbTexture => _cameraSimulator?.RgbTexture;

        /// <summary>
        /// The depth RenderTexture (linear, in meters).
        /// </summary>
        public RenderTexture DepthTexture => _depthPass?.DepthTexture;

        /// <summary>
        /// Native pointer for RGB texture (for CUDA registration).
        /// </summary>
        public IntPtr RgbNativePtr => _cameraSimulator?.NativeTexturePtr ?? IntPtr.Zero;

        /// <summary>
        /// Native pointer for depth texture (for CUDA registration).
        /// </summary>
        public IntPtr DepthNativePtr => _depthPass?.NativeTexturePtr ?? IntPtr.Zero;

        /// <summary>
        /// Whether both RGB and depth are initialized.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Width of the capture textures.
        /// </summary>
        public int Width => _cameraSimulator?.Width ?? 0;

        /// <summary>
        /// Height of the capture textures.
        /// </summary>
        public int Height => _cameraSimulator?.Height ?? 0;

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            _cameraSimulator = GetComponent<CameraSimulator>();
        }

        private void Start()
        {
            if (_autoInitialize && Application.isPlaying)
            {
                Initialize();
            }
        }

        private void OnDestroy()
        {
            Cleanup();
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Initialize the camera capture system.
        /// </summary>
        public bool Initialize()
        {
            if (_isInitialized)
            {
                Debug.LogWarning("[SimulationCameraController] Already initialized");
                return true;
            }

            // Initialize RGB capture
            if (_cameraSimulator == null)
            {
                _cameraSimulator = GetComponent<CameraSimulator>();
            }

            if (_cameraSimulator == null)
            {
                Debug.LogError("[SimulationCameraController] CameraSimulator component not found");
                return false;
            }

            if (!_cameraSimulator.Initialize())
            {
                Debug.LogError("[SimulationCameraController] Failed to initialize CameraSimulator");
                return false;
            }

            // Initialize depth capture
            if (_enableDepth)
            {
                if (!InitializeDepthCapture())
                {
                    Debug.LogWarning("[SimulationCameraController] Depth capture initialization failed, continuing with RGB only");
                }
            }

            _isInitialized = true;
            Debug.Log($"[SimulationCameraController] Initialized: " +
                      $"RGB={_cameraSimulator.Width}x{_cameraSimulator.Height} (0x{RgbNativePtr.ToInt64():X}), " +
                      $"Depth={(_depthPass != null ? $"{_depthPass.width}x{_depthPass.height} (0x{DepthNativePtr.ToInt64():X})" : "disabled")}");

            return true;
        }

        /// <summary>
        /// Cleanup all resources.
        /// </summary>
        public void Cleanup()
        {
            if (_depthPass != null && _customPassVolume != null)
            {
                _customPassVolume.customPasses.Remove(_depthPass);
                _depthPass = null;
            }

            if (_ownsCustomPassVolume && _customPassVolume != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(_customPassVolume.gameObject);
                }
                else
                {
                    DestroyImmediate(_customPassVolume.gameObject);
                }
                _customPassVolume = null;
            }

            _isInitialized = false;
        }

        /// <summary>
        /// Resize both RGB and depth textures.
        /// </summary>
        public bool Resize(int width, int height)
        {
            bool success = true;

            if (_cameraSimulator != null)
            {
                success &= _cameraSimulator.Resize(width, height);
            }

            if (_depthPass != null)
            {
                _depthPass.Resize(width, height);
            }

            return success;
        }

        #endregion

        #region Private Methods

        private bool InitializeDepthCapture()
        {
            if (_linearizeDepthShader == null)
            {
                Debug.LogError("[SimulationCameraController] Linearize depth compute shader not assigned");
                return false;
            }

            // Get or create Custom Pass Volume
            if (_customPassVolume == null)
            {
                _customPassVolume = FindOrCreateCustomPassVolume();
                _ownsCustomPassVolume = true;
            }

            if (_customPassVolume == null)
            {
                Debug.LogError("[SimulationCameraController] Failed to get/create Custom Pass Volume");
                return false;
            }

            // Create and add the depth pass
            _depthPass = new LinearDepthCapturePass
            {
                width = _cameraSimulator.Width,
                height = _cameraSimulator.Height,
                nearClip = _cameraSimulator.Camera.nearClipPlane,
                farClip = _cameraSimulator.Camera.farClipPlane,
                invalidDepthValue = _invalidDepthValue,
                linearizeDepthShader = _linearizeDepthShader,
                targetColorBuffer = CustomPass.TargetBuffer.None,
                targetDepthBuffer = CustomPass.TargetBuffer.None,
                clearFlags = ClearFlag.None,
                enabled = true
            };

            _customPassVolume.customPasses.Add(_depthPass);

            return true;
        }

        private CustomPassVolume FindOrCreateCustomPassVolume()
        {
            // First, try to find an existing global volume
            var existingVolume = FindObjectOfType<CustomPassVolume>();
            if (existingVolume != null && existingVolume.isGlobal)
            {
                return existingVolume;
            }

            // Create a new Custom Pass Volume
            var volumeGO = new GameObject("SimulationDepthCaptureVolume");
            volumeGO.transform.SetParent(transform);

            var volume = volumeGO.AddComponent<CustomPassVolume>();
            volume.isGlobal = true;
            volume.injectionPoint = CustomPassInjectionPoint.AfterOpaqueDepthAndNormal;

            return volume;
        }

        #endregion

        #region Editor Support

#if UNITY_EDITOR
        [ContextMenu("Log Texture Info")]
        private void EditorLogTextureInfo()
        {
            Debug.Log($"[SimulationCameraController] RGB: " +
                      $"{_cameraSimulator?.Width}x{_cameraSimulator?.Height}, " +
                      $"Ptr=0x{RgbNativePtr.ToInt64():X}");

            if (_depthPass != null)
            {
                Debug.Log($"[SimulationCameraController] Depth: " +
                          $"{_depthPass.width}x{_depthPass.height}, " +
                          $"Ptr=0x{DepthNativePtr.ToInt64():X}");
            }
            else
            {
                Debug.Log("[SimulationCameraController] Depth: not initialized");
            }
        }

        [ContextMenu("Reinitialize")]
        private void EditorReinitialize()
        {
            Cleanup();
            Initialize();
        }
#endif

        #endregion
    }
}
