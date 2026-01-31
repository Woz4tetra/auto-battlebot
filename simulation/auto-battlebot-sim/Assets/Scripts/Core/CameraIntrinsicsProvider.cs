// Auto-Battlebot Simulation System
// CameraIntrinsicsProvider - MonoBehaviour that manages camera intrinsics
//
// Attach to a GameObject with a Camera component to configure the camera
// based on intrinsic parameters and expose the CameraIntrinsics struct
// for use by CommunicationBridge and TCP communication.

using System;
using UnityEngine;

namespace AutoBattlebot.Core
{
    /// <summary>
    /// Provides camera intrinsic parameters for simulation.
    /// Attach to a GameObject with a Camera component.
    /// 
    /// This component:
    /// - Stores camera intrinsic parameters (focal length, principal point, distortion)
    /// - Configures the Unity Camera's field of view to match the intrinsics
    /// - Exposes a CameraIntrinsics struct for TCP communication
    /// - Supports ZED 2i camera specifications
    /// 
    /// Usage:
    /// 1. Attach to a GameObject with a Camera
    /// 2. Configure intrinsic parameters in the Inspector (or use defaults)
    /// 3. CommunicationBridge will use these intrinsics for TCP transmission
    /// </summary>
    [RequireComponent(typeof(Camera))]
    [ExecuteInEditMode]
    public class CameraIntrinsicsProvider : MonoBehaviour
    {
        #region Serialized Fields

        [Header("Resolution")]
        [SerializeField]
        [Tooltip("Image width in pixels")]
        private int _width = 1280;

        [SerializeField]
        [Tooltip("Image height in pixels")]
        private int _height = 720;

        [Header("Intrinsic Parameters (ZED 2i defaults for 720p)")]
        [SerializeField]
        [Tooltip("Focal length in X direction (pixels)")]
        private double _fx = 707.6595458984375;

        [SerializeField]
        [Tooltip("Focal length in Y direction (pixels)")]
        private double _fy = 707.6595458984375;

        [SerializeField]
        [Tooltip("Principal point X coordinate (pixels)")]
        private double _cx = 647.5008544921875;

        [SerializeField]
        [Tooltip("Principal point Y coordinate (pixels)")]
        private double _cy = 374.5303039550781;

        [Header("Distortion Coefficients (0 for rectified ZED images)")]
        [SerializeField]
        [Tooltip("Radial distortion k1")]
        private double _k1 = 0;

        [SerializeField]
        [Tooltip("Radial distortion k2")]
        private double _k2 = 0;

        [SerializeField]
        [Tooltip("Tangential distortion p1")]
        private double _p1 = 0;

        [SerializeField]
        [Tooltip("Tangential distortion p2")]
        private double _p2 = 0;

        [SerializeField]
        [Tooltip("Radial distortion k3")]
        private double _k3 = 0;

        [Header("Camera Configuration")]
        [SerializeField]
        [Tooltip("Automatically configure the Unity Camera's FOV from intrinsics")]
        private bool _applyCameraFov = true;

        [SerializeField]
        [Tooltip("Near clip plane (meters) - ZED 2i minimum depth")]
        private float _nearClip = 0.3f;

        [SerializeField]
        [Tooltip("Far clip plane (meters) - ZED 2i maximum depth")]
        private float _farClip = 20f;

        #endregion

        #region Private Fields

        private Camera _camera;
        private CameraIntrinsics _intrinsics;

        #endregion

        #region Properties

        /// <summary>
        /// The camera intrinsics as a struct for TCP communication.
        /// </summary>
        public CameraIntrinsics Intrinsics
        {
            get
            {
                // Rebuild if needed
                if (_intrinsics.Width != _width || _intrinsics.Height != _height)
                {
                    RebuildIntrinsics();
                }
                return _intrinsics;
            }
        }

        /// <summary>
        /// The attached Camera component.
        /// </summary>
        public Camera Camera => _camera;

        /// <summary>
        /// Image width in pixels.
        /// </summary>
        public int Width => _width;

        /// <summary>
        /// Image height in pixels.
        /// </summary>
        public int Height => _height;

        /// <summary>
        /// Focal length X (pixels).
        /// </summary>
        public double Fx => _fx;

        /// <summary>
        /// Focal length Y (pixels).
        /// </summary>
        public double Fy => _fy;

        /// <summary>
        /// Principal point X (pixels).
        /// </summary>
        public double Cx => _cx;

        /// <summary>
        /// Principal point Y (pixels).
        /// </summary>
        public double Cy => _cy;

        /// <summary>
        /// Near clip plane (meters).
        /// </summary>
        public float NearClip => _nearClip;

        /// <summary>
        /// Far clip plane (meters).
        /// </summary>
        public float FarClip => _farClip;

        /// <summary>
        /// Horizontal field of view in degrees (computed from intrinsics).
        /// </summary>
        public float HorizontalFovDegrees
        {
            get
            {
                double fovRadians = 2.0 * Math.Atan(_width / (2.0 * _fx));
                return (float)(fovRadians * 180.0 / Math.PI);
            }
        }

        /// <summary>
        /// Vertical field of view in degrees (computed from intrinsics).
        /// </summary>
        public float VerticalFovDegrees
        {
            get
            {
                double fovRadians = 2.0 * Math.Atan(_height / (2.0 * _fy));
                return (float)(fovRadians * 180.0 / Math.PI);
            }
        }

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            _camera = GetComponent<Camera>();
            RebuildIntrinsics();
        }

        private void Start()
        {
            ApplyToCamera();
        }

        private void OnValidate()
        {
            if (_applyCameraFov && _camera == null)
            {
                _camera = GetComponent<Camera>();
            }

            RebuildIntrinsics();

            if (_applyCameraFov && _camera != null)
            {
                ApplyToCamera();
            }
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Applies the intrinsics to the Unity Camera component.
        /// Sets the camera's field of view based on focal length.
        /// </summary>
        public void ApplyToCamera()
        {
            if (_camera == null)
            {
                _camera = GetComponent<Camera>();
            }

            if (_camera == null)
            {
                return;
            }

            // Set vertical FOV
            _camera.fieldOfView = VerticalFovDegrees;
            _camera.nearClipPlane = _nearClip;
            _camera.farClipPlane = _farClip;

            // Note: Unity doesn't support non-centered principal points natively.
            // For accurate simulation of off-center principal points, a custom
            // projection matrix is applied by CameraSimulator.
        }

        /// <summary>
        /// Sets intrinsics from a CameraIntrinsics struct.
        /// </summary>
        /// <param name="intrinsics">The intrinsics to apply.</param>
        public void SetIntrinsics(CameraIntrinsics intrinsics)
        {
            _width = intrinsics.Width;
            _height = intrinsics.Height;
            _fx = intrinsics.Fx;
            _fy = intrinsics.Fy;
            _cx = intrinsics.Cx;
            _cy = intrinsics.Cy;
            _k1 = intrinsics.K1;
            _k2 = intrinsics.K2;
            _p1 = intrinsics.P1;
            _p2 = intrinsics.P2;
            _k3 = intrinsics.K3;

            _intrinsics = intrinsics;

            if (_applyCameraFov)
            {
                ApplyToCamera();
            }
        }

        /// <summary>
        /// Sets resolution and scales intrinsics proportionally.
        /// </summary>
        /// <param name="newWidth">New width in pixels.</param>
        /// <param name="newHeight">New height in pixels.</param>
        public void SetResolution(int newWidth, int newHeight)
        {
            if (newWidth <= 0 || newHeight <= 0)
            {
                Debug.LogError("[CameraIntrinsicsProvider] Invalid resolution");
                return;
            }

            // Calculate scale factors
            double scaleX = (double)newWidth / _width;
            double scaleY = (double)newHeight / _height;

            // Scale intrinsics
            _fx *= scaleX;
            _fy *= scaleY;
            _cx *= scaleX;
            _cy *= scaleY;

            _width = newWidth;
            _height = newHeight;

            RebuildIntrinsics();

            if (_applyCameraFov)
            {
                ApplyToCamera();
            }
        }

        /// <summary>
        /// Creates intrinsics from the current Unity Camera settings.
        /// </summary>
        public void CreateFromCurrentCamera()
        {
            if (_camera == null)
            {
                _camera = GetComponent<Camera>();
            }

            if (_camera == null)
            {
                Debug.LogError("[CameraIntrinsicsProvider] No camera attached");
                return;
            }

            // Calculate focal length from FOV
            // fy = height / (2 * tan(fov/2))
            double fovRadians = _camera.fieldOfView * Math.PI / 180.0;
            _fy = _height / (2.0 * Math.Tan(fovRadians / 2.0));
            _fx = _fy;  // Assume square pixels

            // Assume centered principal point
            _cx = _width / 2.0;
            _cy = _height / 2.0;

            // No distortion
            _k1 = 0; _k2 = 0; _p1 = 0; _p2 = 0; _k3 = 0;

            RebuildIntrinsics();
        }

        #endregion

        #region Private Methods

        private void RebuildIntrinsics()
        {
            _intrinsics = CameraIntrinsics.Create(
                _width, _height,
                _fx, _fy, _cx, _cy,
                _k1, _k2, _p1, _p2, _k3);
        }

        #endregion

        #region Editor Support

#if UNITY_EDITOR
        [ContextMenu("Apply to Camera")]
        private void EditorApplyToCamera()
        {
            ApplyToCamera();
        }

        [ContextMenu("Create From Current Camera FOV")]
        private void EditorCreateFromCamera()
        {
            CreateFromCurrentCamera();
        }

        [ContextMenu("Log Intrinsics")]
        private void EditorLogIntrinsics()
        {
            Debug.Log($"[CameraIntrinsicsProvider] {Intrinsics}\n" +
                      $"  HFOV: {HorizontalFovDegrees:F1}°, VFOV: {VerticalFovDegrees:F1}°");
        }

        [ContextMenu("Apply ZED 2i HD720 Defaults")]
        private void EditorApplyZed720()
        {
            _width = 1280;
            _height = 720;
            _fx = 707.6595458984375;
            _fy = 707.6595458984375;
            _cx = 647.5008544921875;
            _cy = 374.5303039550781;
            _k1 = 0; _k2 = 0; _p1 = 0; _p2 = 0; _k3 = 0;
            _nearClip = 0.3f;
            _farClip = 20f;
            RebuildIntrinsics();
            ApplyToCamera();
        }

        [ContextMenu("Apply ZED 2i HD1080 Defaults")]
        private void EditorApplyZed1080()
        {
            _width = 1920;
            _height = 1080;
            _fx = 1061.4892578125;
            _fy = 1061.4892578125;
            _cx = 971.2513427734375;
            _cy = 561.7954711914062;
            _k1 = 0; _k2 = 0; _p1 = 0; _p2 = 0; _k3 = 0;
            _nearClip = 0.3f;
            _farClip = 20f;
            RebuildIntrinsics();
            ApplyToCamera();
        }

        [ContextMenu("Apply ZED 2i VGA Defaults")]
        private void EditorApplyZedVGA()
        {
            _width = 640;
            _height = 360;
            _fx = 353.82977294921875;
            _fy = 353.82977294921875;
            _cx = 323.75042724609375;
            _cy = 187.26515197753906;
            _k1 = 0; _k2 = 0; _p1 = 0; _p2 = 0; _k3 = 0;
            _nearClip = 0.3f;
            _farClip = 20f;
            RebuildIntrinsics();
            ApplyToCamera();
        }
#endif

        #endregion
    }
}
