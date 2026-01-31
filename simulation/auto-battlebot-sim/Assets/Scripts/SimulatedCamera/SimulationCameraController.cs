// Auto-Battlebot Simulation System
// SimulationCameraController - Unified controller for RGB and Depth capture
//
// Coordinates CameraSimulator (RGB) and LinearDepthCapturePass (Depth)
// to provide synchronized RGB-D frames for AsyncGPUReadback and TCP transfer.

using System;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using AutoBattlebot.Core;

namespace AutoBattlebot.SimulatedCamera
{
    /// <summary>
    /// Unified controller for simulation camera capture.
    /// 
    /// Coordinates:
    /// - CameraSimulator for RGB capture
    /// - LinearDepthCapturePass for depth capture
    /// - Provides synchronized access to both textures for AsyncGPUReadback
    /// 
    /// This component ensures RGB and depth are aligned and captured at the same resolution.
    /// The CommunicationBridge uses AsyncGPUReadback on both textures and sends them over TCP.
    /// 
    /// Usage:
    /// 1. Attach to the same GameObject as the Camera
    /// 2. Assign the compute shader and (optionally) a Custom Pass Volume
    /// 3. Call Initialize() or rely on auto-initialization
    /// 4. CommunicationBridge will auto-detect and use both textures
    /// </summary>
    [RequireComponent(typeof(CameraSimulator))]
    public class SimulationCameraController : MonoBehaviour
    {
        #region Serialized Fields

        [Header("Depth Capture")]
        [SerializeField]
        [Tooltip("Use blit shader instead of compute (more HDRP compatible)")]
        private bool _useBlitShader = true;

        [SerializeField]
        [Tooltip("Blit shader for depth linearization (HDRP compatible)")]
        private Shader _linearizeDepthBlitShader;

        [SerializeField]
        [Tooltip("Compute shader for depth linearization (alternative)")]
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

        [Header("Depth Noise (Optional)")]
        [SerializeField]
        [Tooltip("Enable depth noise simulation for realism")]
        private bool _enableDepthNoise = false;

        [SerializeField]
        [Tooltip("Base noise level (meters)")]
        [Range(0f, 0.01f)]
        private float _baseNoiseLevel = 0.002f;

        [SerializeField]
        [Tooltip("Distance-dependent noise factor")]
        [Range(0f, 0.01f)]
        private float _distanceNoiseFactor = 0.001f;

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
        /// Native pointer for RGB texture (for debugging/logging).
        /// </summary>
        public IntPtr RgbNativePtr => _cameraSimulator?.NativeTexturePtr ?? IntPtr.Zero;

        /// <summary>
        /// Native pointer for depth texture (for debugging/logging).
        /// </summary>
        public IntPtr DepthNativePtr => _depthPass?.NativeTexturePtr ?? IntPtr.Zero;

        /// <summary>
        /// Whether both RGB and depth are initialized.
        /// </summary>
        public bool IsInitialized => _isInitialized;

        /// <summary>
        /// Whether depth capture is enabled and working.
        /// </summary>
        public bool HasDepth => _depthPass != null && _depthPass.HasTexture;

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
                      $"RGB={_cameraSimulator.Width}x{_cameraSimulator.Height} ({_cameraSimulator.RgbTexture?.graphicsFormat}), " +
                      $"Depth={(_depthPass != null ? $"{_depthPass.width}x{_depthPass.height} ({_depthPass.DepthTexture?.graphicsFormat})" : "disabled")}");

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
        /// Enable or disable depth noise at runtime.
        /// </summary>
        public void SetDepthNoise(bool enabled, float baseNoise = 0.002f, float distanceFactor = 0.001f)
        {
            _enableDepthNoise = enabled;
            _baseNoiseLevel = baseNoise;
            _distanceNoiseFactor = distanceFactor;

            if (_depthPass != null)
            {
                _depthPass.enableNoise = enabled;
                _depthPass.baseNoiseLevel = baseNoise;
                _depthPass.distanceNoiseFactor = distanceFactor;
            }
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

            // Get camera intrinsics for depth configuration
            var intrinsicsProvider = _cameraSimulator.GetComponent<CameraIntrinsicsProvider>();
            float nearClip = intrinsicsProvider?.NearClip ?? _cameraSimulator.Camera.nearClipPlane;
            float farClip = intrinsicsProvider?.FarClip ?? _cameraSimulator.Camera.farClipPlane;

            // Create and add the depth pass
            _depthPass = new LinearDepthCapturePass
            {
                width = _cameraSimulator.Width,
                height = _cameraSimulator.Height,
                nearClip = nearClip,
                farClip = farClip,
                invalidDepthValue = _invalidDepthValue,
                useBlitShader = _useBlitShader,
                linearizeDepthBlitShader = _linearizeDepthBlitShader,
                linearizeDepthShader = _linearizeDepthShader,
                enableNoise = _enableDepthNoise,
                baseNoiseLevel = _baseNoiseLevel,
                distanceNoiseFactor = _distanceNoiseFactor,
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
            var existingVolume = FindFirstObjectByType<CustomPassVolume>();
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
                      $"Format={_cameraSimulator?.RgbTexture?.graphicsFormat}");

            if (_depthPass != null)
            {
                Debug.Log($"[SimulationCameraController] Depth: " +
                          $"{_depthPass.width}x{_depthPass.height}, " +
                          $"Format={_depthPass.DepthTexture?.graphicsFormat}, " +
                          $"Noise={_depthPass.enableNoise}");
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

        [ContextMenu("Toggle Depth Noise")]
        private void EditorToggleDepthNoise()
        {
            SetDepthNoise(!_enableDepthNoise, _baseNoiseLevel, _distanceNoiseFactor);
            Debug.Log($"[SimulationCameraController] Depth noise: {(_enableDepthNoise ? "enabled" : "disabled")}");
        }
#endif

        #endregion
    }
}
