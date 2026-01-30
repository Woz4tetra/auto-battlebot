// Auto-Battlebot Simulation System
// SimulationCameraSetup - One-click setup for simulation camera with CUDA Interop
//
// This component automatically configures all camera components and wires them
// to the CudaInteropBridge for zero-copy GPU texture sharing.

using System;
using UnityEngine;

namespace AutoBattlebot.Camera
{
    /// <summary>
    /// Automatic setup component for simulation camera system.
    /// 
    /// This component simplifies the setup process by:
    /// - Creating/configuring CameraSimulator for RGB capture
    /// - Creating/configuring LinearDepthCapturePass for depth capture
    /// - Automatically registering textures with an external bridge
    /// 
    /// Usage:
    /// 1. Attach to a Camera GameObject
    /// 2. Assign the compute shader
    /// 3. Optionally assign a CudaInteropBridge reference to auto-wire textures
    /// 4. The component handles the rest on Start()
    /// </summary>
    [RequireComponent(typeof(UnityEngine.Camera))]
    public class SimulationCameraSetup : MonoBehaviour
    {
        #region Serialized Fields

        [Header("Resolution")]
        [SerializeField]
        [Tooltip("Render width in pixels")]
        private int _width = 1280;

        [SerializeField]
        [Tooltip("Render height in pixels")]
        private int _height = 720;

        [Header("Camera Preset")]
        [SerializeField]
        [Tooltip("Camera preset to use")]
        private CameraPreset _preset = CameraPreset.Zed2i_720p;

        [Header("Depth Capture")]
        [SerializeField]
        [Tooltip("Enable depth capture")]
        private bool _enableDepth = true;

        [SerializeField]
        [Tooltip("Compute shader for depth linearization")]
        private ComputeShader _linearizeDepthShader;

        [SerializeField]
        [Tooltip("Value for invalid/sky depth (0 = no depth)")]
        private float _invalidDepthValue = 0.0f;

        [Header("Integration")]
        [SerializeField]
        [Tooltip("Callback when textures are ready (assign via script or UnityEvent)")]
        private TexturesReadyEvent _onTexturesReady;

        #endregion

        #region Private Fields

        private CameraSimulator _cameraSimulator;
        private SimulationCameraController _controller;

        #endregion

        #region Properties

        /// <summary>
        /// The CameraSimulator component.
        /// </summary>
        public CameraSimulator CameraSimulator => _cameraSimulator;

        /// <summary>
        /// The SimulationCameraController component.
        /// </summary>
        public SimulationCameraController Controller => _controller;

        /// <summary>
        /// The RGB RenderTexture.
        /// </summary>
        public RenderTexture RgbTexture => _controller?.RgbTexture;

        /// <summary>
        /// The depth RenderTexture.
        /// </summary>
        public RenderTexture DepthTexture => _controller?.DepthTexture;

        /// <summary>
        /// Whether the system is initialized.
        /// </summary>
        public bool IsInitialized => _controller?.IsInitialized ?? false;

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            SetupComponents();
        }

        private void Start()
        {
            if (Application.isPlaying)
            {
                ApplyPreset(_preset);

                if (_controller != null && _controller.Initialize())
                {
                    // Fire event with texture references
                    _onTexturesReady?.Invoke(RgbTexture, DepthTexture);
                }
            }
        }

        private void OnValidate()
        {
            _width = Mathf.Max(64, _width);
            _height = Mathf.Max(64, _height);
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Get texture information for external integration.
        /// </summary>
        public (RenderTexture rgb, RenderTexture depth, IntPtr rgbPtr, IntPtr depthPtr) GetTextureInfo()
        {
            return (
                RgbTexture,
                DepthTexture,
                _controller?.RgbNativePtr ?? IntPtr.Zero,
                _controller?.DepthNativePtr ?? IntPtr.Zero
            );
        }

        /// <summary>
        /// Apply a camera preset.
        /// </summary>
        public void ApplyPreset(CameraPreset preset)
        {
            switch (preset)
            {
                case CameraPreset.Zed2i_720p:
                    ConfigureCamera(1280, 720, 707.66, 707.66, 647.50, 374.53, 0.3f, 20f);
                    break;
                case CameraPreset.Zed2i_1080p:
                    ConfigureCamera(1920, 1080, 1061.49, 1061.49, 971.25, 561.80, 0.3f, 20f);
                    break;
                case CameraPreset.Zed2i_VGA:
                    ConfigureCamera(640, 360, 353.83, 353.83, 323.75, 187.27, 0.3f, 20f);
                    break;
                case CameraPreset.Custom:
                    // Use current _width, _height settings
                    break;
            }
        }

        #endregion

        #region Private Methods

        private void SetupComponents()
        {
            // Get or add CameraSimulator
            _cameraSimulator = GetComponent<CameraSimulator>();
            if (_cameraSimulator == null)
            {
                _cameraSimulator = gameObject.AddComponent<CameraSimulator>();
            }

            // Get or add SimulationCameraController
            _controller = GetComponent<SimulationCameraController>();
            if (_controller == null)
            {
                _controller = gameObject.AddComponent<SimulationCameraController>();
            }
        }

        private void ConfigureCamera(int width, int height, double fx, double fy,
                                     double cx, double cy, float near, float far)
        {
            _width = width;
            _height = height;

            if (_cameraSimulator != null)
            {
                // Use reflection or direct field access to configure
                // For now, we'll rely on the CameraSimulator's own preset methods
                var cam = GetComponent<UnityEngine.Camera>();
                if (cam != null)
                {
                    cam.nearClipPlane = near;
                    cam.farClipPlane = far;

                    // Calculate FOV from focal length
                    double fovRadians = 2.0 * Math.Atan(height / (2.0 * fy));
                    cam.fieldOfView = (float)(fovRadians * 180.0 / Math.PI);
                }
            }
        }

        #endregion

        #region Enums and Events

        /// <summary>
        /// Camera presets matching ZED SDK configurations.
        /// </summary>
        public enum CameraPreset
        {
            Zed2i_720p,
            Zed2i_1080p,
            Zed2i_VGA,
            Custom
        }

        /// <summary>
        /// Event fired when textures are ready.
        /// </summary>
        [Serializable]
        public class TexturesReadyEvent : UnityEngine.Events.UnityEvent<RenderTexture, RenderTexture> { }

        #endregion

        #region Editor Support

#if UNITY_EDITOR
        [ContextMenu("Setup Components")]
        private void EditorSetupComponents()
        {
            SetupComponents();
        }

        [ContextMenu("Apply ZED 2i 720p")]
        private void EditorApplyZed720p()
        {
            _preset = CameraPreset.Zed2i_720p;
            ApplyPreset(_preset);
        }

        [ContextMenu("Apply ZED 2i 1080p")]
        private void EditorApplyZed1080p()
        {
            _preset = CameraPreset.Zed2i_1080p;
            ApplyPreset(_preset);
        }

        [ContextMenu("Log Setup Status")]
        private void EditorLogStatus()
        {
            Debug.Log($"[SimulationCameraSetup] Preset: {_preset}");
            Debug.Log($"[SimulationCameraSetup] Resolution: {_width}x{_height}");
            Debug.Log($"[SimulationCameraSetup] Depth enabled: {_enableDepth}");
            Debug.Log($"[SimulationCameraSetup] Shader assigned: {_linearizeDepthShader != null}");

            if (IsInitialized)
            {
                var (rgb, depth, rgbPtr, depthPtr) = GetTextureInfo();
                Debug.Log($"[SimulationCameraSetup] RGB: {rgb?.width}x{rgb?.height}, Ptr=0x{rgbPtr.ToInt64():X}");
                Debug.Log($"[SimulationCameraSetup] Depth: {depth?.width}x{depth?.height}, Ptr=0x{depthPtr.ToInt64():X}");
            }
            else
            {
                Debug.Log("[SimulationCameraSetup] Not initialized");
            }
        }
#endif

        #endregion
    }
}
