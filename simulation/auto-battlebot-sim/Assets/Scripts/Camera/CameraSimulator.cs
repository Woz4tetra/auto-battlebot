// Auto-Battlebot Simulation System
// Virtual camera simulator matching ZED 2i characteristics

using System;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using AutoBattlebot.Core;
using AutoBattlebot.Communication;

namespace AutoBattlebot.Camera
{
    using UnityCamera = UnityEngine.Camera;
    /// <summary>
    /// Simulates the Stereolabs ZED 2i camera and streams RGB frames.
    /// Uses AsyncGPUReadback to minimize CPU stalls.
    /// </summary>
    public class CameraSimulator : MonoBehaviour, IInitializable
    {
        #region Serialized Fields

        [Header("Camera Settings")]
        [SerializeField]
        [Tooltip("Camera component used for rendering")]
        private UnityCamera _camera;

        [SerializeField]
        [Tooltip("Width of the captured images")]
        private int _imageWidth = 1280;

        [SerializeField]
        [Tooltip("Height of the captured images")]
        private int _imageHeight = 720;

        [SerializeField]
        [Tooltip("Target capture frame rate")]
        private int _captureFrameRate = 30;

        [SerializeField]
        [Tooltip("Vertical field of view in degrees (Unity Camera.fieldOfView)")]
        private float _verticalFovDegrees = 70f;

        [Header("ZED 2i Intrinsics (1080p base)")]
        [SerializeField]
        [Tooltip("Use ZED 2i intrinsics scaled to the selected resolution")]
        private bool _useZedIntrinsics = true;

        [SerializeField]
        private double _zedFx1080p = 1061.4892578125;

        [SerializeField]
        private double _zedFy1080p = 1061.4892578125;

        [SerializeField]
        private double _zedCx1080p = 971.2513427734375;

        [SerializeField]
        private double _zedCy1080p = 561.7954711914062;

        [Header("Distortion (Brown-Conrady)")]
        [SerializeField]
        [Tooltip("Apply lens distortion to the output buffer")]
        private bool _applyDistortion = false;

        [SerializeField] private double _k1 = 0;
        [SerializeField] private double _k2 = 0;
        [SerializeField] private double _p1 = 0;
        [SerializeField] private double _p2 = 0;
        [SerializeField] private double _k3 = 0;

        [Header("Communication")]
        [SerializeField]
        [Tooltip("Optional CommunicationBridge to write frames to shared memory")]
        private CommunicationBridge _communicationBridge;

        #endregion

        #region Properties

        public int ImageWidth => _imageWidth;
        public int ImageHeight => _imageHeight;
        public int CaptureFrameRate => _captureFrameRate;

        #endregion

        #region Private Fields

        private RenderTexture _colorTarget;
        private RenderTexture _depthTarget;
        private bool _colorReadbackInFlight;
        private bool _depthReadbackInFlight;
        private float _nextCaptureTime;
        private bool _cameraModelApplied;

        private byte[] _bgrBuffer;
        private byte[] _distortedBgrBuffer;
        private float[] _depthBuffer;
        private bool _colorReady;
        private bool _depthReady;

        private double _fx;
        private double _fy;
        private double _cx;
        private double _cy;

        // Near/far planes for depth conversion
        private float _nearClip;
        private float _farClip;

        // Depth capture material (unused but kept for potential future use)
        private Material _depthCopyMaterial;

        #endregion

        #region IInitializable Implementation

        public InitializationPhase Phase => InitializationPhase.Init;

        public void Initialize()
        {
            if (_camera == null)
            {
                _camera = GetComponent<UnityCamera>();
            }
            if (_camera == null)
            {
                _camera = GetComponentInChildren<UnityCamera>();
            }
            if (_camera == null)
            {
                _camera = FindFirstObjectByType<UnityCamera>();
            }

            if (_camera == null)
            {
                Debug.LogError($"[CameraSimulator] No Camera component assigned on {name}. " +
                               "Assign a Camera in the inspector or place one on this GameObject.");
                return;
            }

            Debug.Log($"[CameraSimulator] Initializing camera ({_imageWidth}x{_imageHeight} @ {_captureFrameRate}fps)...");
            if (!SystemInfo.supportsAsyncGPUReadback)
            {
                Debug.LogWarning("[CameraSimulator] AsyncGPUReadback not supported on this device");
            }
            ConfigureCamera();
            AllocateBuffers();
            ConfigureCommunicationBridge();
            Debug.Log("[CameraSimulator] Camera ready");
        }

        public void Shutdown()
        {
            Debug.Log("[CameraSimulator] Releasing camera resources...");
            ReleaseResources();
        }

        #endregion

        #region Unity Lifecycle

        private void Update()
        {
            if (_camera == null || _colorTarget == null)
            {
                return;
            }

            if (!_cameraModelApplied)
            {
                TryApplyCameraModel();
            }

            if (_captureFrameRate > 0 && Time.time < _nextCaptureTime)
            {
                return;
            }

            if (_colorReadbackInFlight || _depthReadbackInFlight)
            {
                return;
            }

            _nextCaptureTime = _captureFrameRate > 0 ? Time.time + (1f / _captureFrameRate) : Time.time;
            CaptureFrame();
        }

        private void OnDestroy()
        {
            ReleaseResources();
        }

        #endregion

        #region Private Methods

        private void ConfigureCamera()
        {
            _camera.fieldOfView = _verticalFovDegrees;
            _camera.aspect = _imageWidth / (float)_imageHeight;
            _camera.forceIntoRenderTexture = true;
            _camera.depthTextureMode = DepthTextureMode.Depth;

            // Store clip planes for depth conversion
            _nearClip = _camera.nearClipPlane;
            _farClip = _camera.farClipPlane;

            if (_colorTarget == null)
            {
                // Create color target with depth buffer attached
                _colorTarget = new RenderTexture(_imageWidth, _imageHeight, 24, RenderTextureFormat.ARGB32)
                {
                    name = "CameraSimulator_Color",
                    useMipMap = false,
                    autoGenerateMips = false
                };
                _colorTarget.Create();
            }

            if (_depthTarget == null)
            {
                // Create readable depth target (RFloat for GPU readback)
                _depthTarget = new RenderTexture(_imageWidth, _imageHeight, 0, RenderTextureFormat.RFloat)
                {
                    name = "CameraSimulator_Depth",
                    useMipMap = false,
                    autoGenerateMips = false,
                    enableRandomWrite = true
                };
                _depthTarget.Create();
            }

            // Create depth copy material from shader source
            if (_depthCopyMaterial == null)
            {
                var shader = Shader.Find("Hidden/Internal-DepthNormalsTexture");
                if (shader == null)
                {
                    // Fallback: try to find any depth shader
                    shader = Shader.Find("Hidden/Camera/CopyDepth");
                }
                if (shader == null)
                {
                    Debug.LogWarning("[CameraSimulator] Could not find depth copy shader. Using fallback depth.");
                }
                else
                {
                    _depthCopyMaterial = new Material(shader);
                }
            }

            _camera.targetTexture = _colorTarget;

            // Subscribe to render pipeline events for depth capture
            RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
            RenderPipelineManager.endCameraRendering += OnEndCameraRendering;

            if (_useZedIntrinsics)
            {
                const double baseWidth = 1920.0;
                const double baseHeight = 1080.0;
                double scaleX = _imageWidth / baseWidth;
                double scaleY = _imageHeight / baseHeight;

                _fx = _zedFx1080p * scaleX;
                _fy = _zedFy1080p * scaleY;
                _cx = _zedCx1080p * scaleX;
                _cy = _zedCy1080p * scaleY;
            }
            else
            {
                double fovRadians = _verticalFovDegrees * Math.PI / 180.0;
                _fy = _imageHeight / (2.0 * Math.Tan(fovRadians / 2.0));
                _fx = _fy;
                _cx = _imageWidth / 2.0;
                _cy = _imageHeight / 2.0;
            }
        }

        private void OnEndCameraRendering(ScriptableRenderContext context, UnityCamera cam)
        {
            if (cam != _camera || _depthTarget == null)
            {
                return;
            }

            // Try to copy depth texture after rendering completes
            var depthTexture = Shader.GetGlobalTexture("_CameraDepthTexture");
            if (depthTexture != null)
            {
                Graphics.Blit(depthTexture, _depthTarget);
            }
        }

        private void AllocateBuffers()
        {
            int pixelCount = _imageWidth * _imageHeight;
            _bgrBuffer = new byte[pixelCount * 3];
            _depthBuffer = new float[pixelCount];
            if (_applyDistortion)
            {
                _distortedBgrBuffer = new byte[pixelCount * 3];
            }
        }

        private void ConfigureCommunicationBridge()
        {
            if (_communicationBridge == null)
            {
                _communicationBridge = FindFirstObjectByType<CommunicationBridge>();
            }

            if (_communicationBridge == null)
            {
                Debug.LogWarning("[CameraSimulator] No CommunicationBridge found. Frames will not be written.");
                return;
            }
            TryApplyCameraModel();
        }

        private void TryApplyCameraModel()
        {
            if (_communicationBridge == null || _communicationBridge.FrameWriter == null)
            {
                return;
            }

            _communicationBridge.FrameWriter.SetCameraModel(
                _fx, _fy, _cx, _cy,
                _k1, _k2, _p1, _p2, _k3);
            _cameraModelApplied = true;
        }

        private void CaptureFrame()
        {
            _camera.Render();

            // Depth is captured in OnEndCameraRendering callback after full render completes
            // Request readbacks for both color and depth
            _colorReadbackInFlight = true;
            _depthReadbackInFlight = true;
            _colorReady = false;
            _depthReady = false;

            AsyncGPUReadback.Request(_colorTarget, 0, TextureFormat.RGBA32, OnColorReadbackComplete);
            AsyncGPUReadback.Request(_depthTarget, 0, TextureFormat.RFloat, OnDepthReadbackComplete);
        }

        private void OnColorReadbackComplete(AsyncGPUReadbackRequest request)
        {
            _colorReadbackInFlight = false;

            // Check if component/buffers are still valid (may be destroyed during shutdown)
            if (_bgrBuffer == null || _camera == null)
            {
                return;
            }

            if (request.hasError)
            {
                Debug.LogWarning("[CameraSimulator] Color GPU readback error");
                return;
            }

            var data = request.GetData<byte>();
            ConvertRgbaToBgr(data, _bgrBuffer);
            _colorReady = true;

            TryWriteFrame();
        }

        private void OnDepthReadbackComplete(AsyncGPUReadbackRequest request)
        {
            _depthReadbackInFlight = false;

            // Check if component/buffers are still valid (may be destroyed during shutdown)
            if (_depthBuffer == null || _camera == null)
            {
                return;
            }

            if (request.hasError)
            {
                Debug.LogWarning("[CameraSimulator] Depth GPU readback error");
                return;
            }

            var data = request.GetData<float>();
            ConvertDepthToMetric(data, _depthBuffer);
            _depthReady = true;

            TryWriteFrame();
        }

        private void TryWriteFrame()
        {
            if (!_colorReady || !_depthReady)
            {
                return;
            }

            byte[] outputBuffer = _bgrBuffer;
            if (_applyDistortion && _distortedBgrBuffer != null)
            {
                ApplyDistortion(_bgrBuffer, _distortedBgrBuffer);
                outputBuffer = _distortedBgrBuffer;
            }

            if (_communicationBridge != null && _communicationBridge.IsActive)
            {
                Matrix4x4 pose = _camera.transform.localToWorldMatrix;
                _communicationBridge.WriteFrame(outputBuffer, _depthBuffer, ConvertMatrixToRowMajor(pose));
            }

            _colorReady = false;
            _depthReady = false;
        }

        private void ConvertDepthToMetric(NativeArray<float> rawDepth, float[] metricDepth)
        {
            int width = _imageWidth;
            int height = _imageHeight;

            // Flip vertically (same as color) and convert from normalized depth to metric
            for (int y = 0; y < height; y++)
            {
                int srcRow = (height - 1 - y) * width;
                int dstRow = y * width;

                for (int x = 0; x < width; x++)
                {
                    float rawValue = rawDepth[srcRow + x];

                    // Unity depth buffer is non-linear: z_buffer = (1/z - 1/far) / (1/near - 1/far)
                    // Solve for z: z = 1 / (z_buffer * (1/near - 1/far) + 1/far)
                    if (rawValue <= 0f || rawValue >= 1f)
                    {
                        metricDepth[dstRow + x] = _farClip;
                    }
                    else
                    {
                        float invNear = 1f / _nearClip;
                        float invFar = 1f / _farClip;
                        float linearDepth = 1f / (rawValue * (invNear - invFar) + invFar);
                        metricDepth[dstRow + x] = linearDepth;
                    }
                }
            }
        }

        private void ConvertRgbaToBgr(NativeArray<byte> rgbaData, byte[] bgrOutput)
        {
            int width = _imageWidth;
            int height = _imageHeight;

            // Flip vertically: Unity uses bottom-left origin, OpenCV uses top-left
            for (int y = 0; y < height; y++)
            {
                int srcRow = (height - 1 - y) * width * 4;  // Read from bottom
                int dstRow = y * width * 3;                  // Write to top

                for (int x = 0; x < width; x++)
                {
                    int srcIndex = srcRow + x * 4;
                    int dstIndex = dstRow + x * 3;

                    bgrOutput[dstIndex] = rgbaData[srcIndex + 2];     // B
                    bgrOutput[dstIndex + 1] = rgbaData[srcIndex + 1]; // G
                    bgrOutput[dstIndex + 2] = rgbaData[srcIndex];     // R
                }
            }
        }

        private void ApplyDistortion(byte[] sourceBgr, byte[] destBgr)
        {
            Array.Clear(destBgr, 0, destBgr.Length);

            int width = _imageWidth;
            int height = _imageHeight;

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    double nx = (x - _cx) / _fx;
                    double ny = (y - _cy) / _fy;

                    double r2 = nx * nx + ny * ny;
                    double r4 = r2 * r2;
                    double r6 = r4 * r2;

                    double radial = 1.0 + _k1 * r2 + _k2 * r4 + _k3 * r6;
                    double xDist = nx * radial + 2.0 * _p1 * nx * ny + _p2 * (r2 + 2.0 * nx * nx);
                    double yDist = ny * radial + _p1 * (r2 + 2.0 * ny * ny) + 2.0 * _p2 * nx * ny;

                    int srcX = (int)Math.Round(_fx * xDist + _cx);
                    int srcY = (int)Math.Round(_fy * yDist + _cy);

                    if (srcX < 0 || srcX >= width || srcY < 0 || srcY >= height)
                    {
                        continue;
                    }

                    int srcIndex = (srcY * width + srcX) * 3;
                    int dstIndex = (y * width + x) * 3;

                    destBgr[dstIndex] = sourceBgr[srcIndex];
                    destBgr[dstIndex + 1] = sourceBgr[srcIndex + 1];
                    destBgr[dstIndex + 2] = sourceBgr[srcIndex + 2];
                }
            }
        }

        private double[] ConvertMatrixToRowMajor(Matrix4x4 matrix)
        {
            return new[]
            {
                (double)matrix.m00, matrix.m01, matrix.m02, matrix.m03,
                matrix.m10, matrix.m11, matrix.m12, matrix.m13,
                matrix.m20, matrix.m21, matrix.m22, matrix.m23,
                matrix.m30, matrix.m31, matrix.m32, matrix.m33
            };
        }

        private void ReleaseResources()
        {
            // Unsubscribe from render pipeline events
            RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;

            if (_colorTarget != null)
            {
                if (_camera != null && _camera.targetTexture == _colorTarget)
                {
                    _camera.targetTexture = null;
                }
                _colorTarget.Release();
                Destroy(_colorTarget);
                _colorTarget = null;
            }

            if (_depthTarget != null)
            {
                _depthTarget.Release();
                Destroy(_depthTarget);
                _depthTarget = null;
            }

            if (_depthCopyMaterial != null)
            {
                Destroy(_depthCopyMaterial);
                _depthCopyMaterial = null;
            }

            _bgrBuffer = null;
            _distortedBgrBuffer = null;
            _depthBuffer = null;
            _colorReadbackInFlight = false;
            _depthReadbackInFlight = false;
        }

        private void OnValidate()
        {
            if (_imageWidth < 1) _imageWidth = 1;
            if (_imageHeight < 1) _imageHeight = 1;
            if (_camera == null)
            {
                _camera = GetComponent<UnityCamera>();
            }
        }

        #endregion
    }
}
