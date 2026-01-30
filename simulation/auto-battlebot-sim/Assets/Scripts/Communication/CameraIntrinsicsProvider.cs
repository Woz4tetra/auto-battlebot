// Auto-Battlebot Simulation System
// CameraIntrinsicsProvider - MonoBehaviour that manages camera intrinsics
//
// Attach to a GameObject with a Camera component to configure the camera
// based on intrinsic parameters and expose the CameraIntrinsics struct
// for use by CudaInteropBridge and other components.

using System;
using UnityEngine;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Provides camera intrinsic parameters for simulation.
    /// Attach to a GameObject with a Camera component.
    /// 
    /// This component:
    /// - Stores camera intrinsic parameters (focal length, principal point, distortion)
    /// - Configures the Unity Camera's field of view to match the intrinsics
    /// - Exposes a CameraIntrinsics struct for TCP communication
    /// 
    /// Usage:
    /// 1. Attach to a GameObject with a Camera
    /// 2. Configure intrinsic parameters in the Inspector
    /// 3. Reference this component from CudaInteropBridge
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

        [Header("Intrinsic Parameters")]
        [SerializeField]
        [Tooltip("Focal length in X direction (pixels)")]
        private double _fx = 707.66;

        [SerializeField]
        [Tooltip("Focal length in Y direction (pixels)")]
        private double _fy = 707.66;

        [SerializeField]
        [Tooltip("Principal point X coordinate (pixels)")]
        private double _cx = 647.50;

        [SerializeField]
        [Tooltip("Principal point Y coordinate (pixels)")]
        private double _cy = 374.53;

        [Header("Distortion Coefficients")]
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
        [Tooltip("Near clip plane (meters)")]
        private float _nearClip = 0.3f;

        [SerializeField]
        [Tooltip("Far clip plane (meters)")]
        private float _farClip = 20f;

        #endregion

        #region Private Fields

        private Camera _camera;
        private CameraIntrinsics _cachedIntrinsics;
        private bool _intrinsicsDirty = true;

        #endregion

        #region Properties

        /// <summary>
        /// The camera intrinsics as a struct for TCP communication.
        /// </summary>
        public CameraIntrinsics Intrinsics
        {
            get
            {
                if (_intrinsicsDirty)
                {
                    UpdateCachedIntrinsics();
                }
                return _cachedIntrinsics;
            }
        }

        /// <summary>
        /// The attached Camera component.
        /// </summary>
        public Camera Camera => _camera;

        /// <summary>
        /// Image width in pixels.
        /// </summary>
        public int Width
        {
            get => _width;
            set { _width = value; _intrinsicsDirty = true; }
        }

        /// <summary>
        /// Image height in pixels.
        /// </summary>
        public int Height
        {
            get => _height;
            set { _height = value; _intrinsicsDirty = true; }
        }

        /// <summary>
        /// Focal length X (pixels).
        /// </summary>
        public double Fx
        {
            get => _fx;
            set { _fx = value; _intrinsicsDirty = true; }
        }

        /// <summary>
        /// Focal length Y (pixels).
        /// </summary>
        public double Fy
        {
            get => _fy;
            set { _fy = value; _intrinsicsDirty = true; }
        }

        /// <summary>
        /// Principal point X (pixels).
        /// </summary>
        public double Cx
        {
            get => _cx;
            set { _cx = value; _intrinsicsDirty = true; }
        }

        /// <summary>
        /// Principal point Y (pixels).
        /// </summary>
        public double Cy
        {
            get => _cy;
            set { _cy = value; _intrinsicsDirty = true; }
        }

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            _camera = GetComponent<Camera>();
        }

        private void Start()
        {
            ApplyToCamera();
        }

        private void OnValidate()
        {
            _intrinsicsDirty = true;

            if (_applyCameraFov && _camera == null)
            {
                _camera = GetComponent<Camera>();
            }

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

            // Calculate vertical FOV from focal length
            // fov = 2 * atan(height / (2 * fy))
            double fovRadians = 2.0 * Math.Atan(_height / (2.0 * _fy));
            float fovDegrees = (float)(fovRadians * 180.0 / Math.PI);

            _camera.fieldOfView = fovDegrees;
            _camera.nearClipPlane = _nearClip;
            _camera.farClipPlane = _farClip;

            // Note: Unity doesn't support non-centered principal points natively.
            // For accurate simulation of off-center principal points, a custom
            // projection matrix would be needed via camera.projectionMatrix.
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

            _intrinsicsDirty = true;

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

            _intrinsicsDirty = true;
        }

        #endregion

        #region Private Methods

        private void UpdateCachedIntrinsics()
        {
            _cachedIntrinsics = CameraIntrinsics.Create(
                _width, _height,
                _fx, _fy, _cx, _cy,
                _k1, _k2, _p1, _p2, _k3);

            _intrinsicsDirty = false;
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
            Debug.Log($"[CameraIntrinsicsProvider] {Intrinsics}");
        }
#endif

        #endregion
    }
}
