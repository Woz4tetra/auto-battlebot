// Auto-Battlebot Simulation System
// Virtual camera simulator matching ZED 2i characteristics

using System;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using AutoBattlebot.Core;
using AutoBattlebot.Communication;

#if UNITY_HDRP
using UnityEngine.Rendering.HighDefinition;
#endif

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

        [Header("Depth Settings")]
        [SerializeField]
        [Tooltip("Minimum depth value in meters. Closer objects are clamped to this value.")]
        private float _minDepth = 0.3f;

        [SerializeField]
        [Tooltip("Maximum depth value in meters. Farther objects are clamped to this value.")]
        private float _maxDepth = 20f;

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

        [Header("Performance")]
        [SerializeField]
        [Tooltip("Use synchronous GPU readback for lower latency (but blocks main thread). Reduces latency from ~200ms to ~50ms.")]
        private bool _useLowLatencyMode = false;

        [SerializeField]
        [Tooltip("Wait for frame requests from C++ before capturing. If disabled, captures at fixed frame rate.")]
        private bool _useRequestDrivenCapture = false;

        #endregion

        #region Properties

        public int ImageWidth => _imageWidth;
        public int ImageHeight => _imageHeight;
        public int CaptureFrameRate => _captureFrameRate;
        public float MinDepth => _minDepth;
        public float MaxDepth => _maxDepth;

        /// <summary>
        /// The RenderTexture used for depth capture. Used by HDRPDepthCopyPass.
        /// </summary>
        public RenderTexture DepthTarget => _depthTarget;

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

        // Depth capture material
        private Material _depthCopyMaterial;

        // Command buffer for depth capture
        private CommandBuffer _depthCommandBuffer;
        private bool _useCommandBufferDepth;

        // Synchronous readback textures (for low-latency mode)
        private Texture2D _syncColorTexture;
        private Texture2D _syncDepthTexture;

        // Request-driven capture state
        private bool _captureDepthThisFrame = true;  // Whether to capture depth for current frame

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
            Debug.Log($"[CameraSimulator] Render Pipeline: {(GraphicsSettings.currentRenderPipeline != null ? GraphicsSettings.currentRenderPipeline.name : "Built-in")}");
            if (!SystemInfo.supportsAsyncGPUReadback)
            {
                Debug.LogWarning("[CameraSimulator] AsyncGPUReadback not supported on this device");
            }
            ConfigureCamera();
            AllocateBuffers();
            ConfigureCommunicationBridge();
            Debug.Log($"[CameraSimulator] Camera ready (near={_nearClip}, far={_farClip})");
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

            // Check if we're waiting for readback to complete
            if (_colorReadbackInFlight || _depthReadbackInFlight)
            {
                return;
            }

            // In request-driven mode, only capture when C++ requests a frame
            if (_useRequestDrivenCapture)
            {
                if (_communicationBridge != null && _communicationBridge.HasPendingFrameRequest)
                {
                    _captureDepthThisFrame = _communicationBridge.PendingRequestIncludesDepth;
                    CaptureFrame();
                }
                return;
            }

            // Standard frame-rate limited capture
            if (_captureFrameRate > 0 && Time.time < _nextCaptureTime)
            {
                return;
            }

            _nextCaptureTime = _captureFrameRate > 0 ? Time.time + (1f / _captureFrameRate) : Time.time;
            _captureDepthThisFrame = true;  // Always capture depth in free-running mode
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

            _camera.targetTexture = _colorTarget;

            // Set up depth capture using CommandBuffer (more reliable across render pipelines)
            SetupDepthCapture();

            // Subscribe to render pipeline events as fallback for depth capture
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
            // This callback is kept for potential future use with SRP depth capture
            // Currently, depth capture is handled in CaptureFrame()
        }

        private void FillFallbackDepth()
        {
            // Fill with max depth as fallback (indicates no valid depth data)
            for (int i = 0; i < _depthBuffer.Length; i++)
            {
                _depthBuffer[i] = _maxDepth;
            }
        }

        private void SetupDepthCapture()
        {
            // Remove existing command buffer if any
            if (_depthCommandBuffer != null)
            {
                _camera.RemoveCommandBuffer(CameraEvent.AfterDepthTexture, _depthCommandBuffer);
                _depthCommandBuffer.Dispose();
                _depthCommandBuffer = null;
            }

            // Check if we're using HDRP
            bool isHDRP = GraphicsSettings.currentRenderPipeline != null &&
                          GraphicsSettings.currentRenderPipeline.GetType().Name.Contains("HDRenderPipeline");

            if (isHDRP)
            {
                SetupHDRPDepthCapture();
                return;
            }

            // Built-in or URP pipeline - use command buffer approach
            string[] depthShaderNames = new[]
            {
                "Hidden/Internal-DepthNormalsTexture",
                "Hidden/Camera/CopyDepth",
                "Hidden/BlitCopy"
            };

            Shader shader = null;
            foreach (var name in depthShaderNames)
            {
                shader = Shader.Find(name);
                if (shader != null && shader.isSupported)
                {
                    Debug.Log($"[CameraSimulator] Found depth shader: {name}");
                    break;
                }
                shader = null;
            }

            if (shader != null)
            {
                _depthCopyMaterial = new Material(shader);

                // Create command buffer to copy depth
                _depthCommandBuffer = new CommandBuffer();
                _depthCommandBuffer.name = "CameraSimulator Depth Copy";
                _depthCommandBuffer.Blit(BuiltinRenderTextureType.Depth, _depthTarget);

                try
                {
                    _camera.AddCommandBuffer(CameraEvent.AfterDepthTexture, _depthCommandBuffer);
                    _useCommandBufferDepth = true;
                    Debug.Log("[CameraSimulator] Using CommandBuffer for depth capture (AfterDepthTexture)");
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[CameraSimulator] Could not add command buffer: {e.Message}");
                    _useCommandBufferDepth = false;
                }
            }
            else
            {
                Debug.LogWarning("[CameraSimulator] No depth shader found. Depth data may be unavailable.");
                _useCommandBufferDepth = false;
            }
        }

        private void SetupHDRPDepthCapture()
        {
            Debug.Log("[CameraSimulator] Setting up HDRP depth capture...");

            // Method 1: Try to find/create Custom Pass Volume
            var customPassType = System.Type.GetType("AutoBattlebot.Camera.HDRPDepthCopyPass, Assembly-CSharp");
            if (customPassType != null)
            {
                var existingVolume = FindCustomPassVolumeWithDepthCopy();
                if (existingVolume != null)
                {
                    Debug.Log("[CameraSimulator] Found existing HDRP Custom Pass Volume with depth copy");
                    _useCommandBufferDepth = true;
                    return;
                }

                try
                {
                    CreateHDRPCustomPassVolume();
                    _useCommandBufferDepth = true;
                    Debug.Log("[CameraSimulator] Created HDRP Custom Pass Volume for depth capture");
                    return;
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[CameraSimulator] Auto-setup failed: {e.Message}");
                }
            }

            // Method 2: Use render pipeline callback to capture depth
            Debug.Log("[CameraSimulator] Using RenderPipelineManager callback for HDRP depth");
            _useCommandBufferDepth = false;

            // Subscribe to end of frame rendering to capture depth
            RenderPipelineManager.endContextRendering -= OnEndContextRendering;
            RenderPipelineManager.endContextRendering += OnEndContextRendering;

            Debug.LogWarning("[CameraSimulator] HDRP depth capture: For best results, add a Custom Pass Volume with HDRPDepthCopyPass to your scene.\n" +
                           "Steps:\n" +
                           "1. Create empty GameObject\n" +
                           "2. Add 'Custom Pass Volume' component\n" +
                           "3. Set Mode = Global, Injection Point = After Opaque Depth And Normal\n" +
                           "4. Add 'HDRPDepthCopyPass' pass");
        }

        private void OnEndContextRendering(ScriptableRenderContext context, System.Collections.Generic.List<UnityCamera> cameras)
        {
            // This is called after HDRP finishes rendering all cameras
            // Try to capture depth texture here
            if (_depthTarget == null || !_depthReadbackInFlight)
            {
                return;
            }

            // In HDRP, _CameraDepthTexture should be available after rendering
            var depthTexture = Shader.GetGlobalTexture("_CameraDepthTexture");
            if (depthTexture != null)
            {
                Graphics.Blit(depthTexture, _depthTarget);
            }
        }

        private GameObject FindCustomPassVolumeWithDepthCopy()
        {
            // Find all CustomPassVolume components in the scene
            var volumes = FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None);
            foreach (var volume in volumes)
            {
                if (volume.GetType().Name == "CustomPassVolume")
                {
                    // Check if it has our depth copy pass
                    var customPassesField = volume.GetType().GetField("customPasses",
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    if (customPassesField != null)
                    {
                        var passes = customPassesField.GetValue(volume) as System.Collections.IList;
                        if (passes != null)
                        {
                            foreach (var pass in passes)
                            {
                                if (pass != null && pass.GetType().Name == "HDRPDepthCopyPass")
                                {
                                    // Set the target texture
                                    var targetField = pass.GetType().GetField("targetDepthTexture");
                                    if (targetField != null)
                                    {
                                        targetField.SetValue(pass, _depthTarget);
                                    }
                                    return volume.gameObject;
                                }
                            }
                        }
                    }
                }
            }
            return null;
        }

        private void CreateHDRPCustomPassVolume()
        {
            // This requires HDRP assembly reference - use reflection to avoid compile errors
            var customPassVolumeType = System.Type.GetType("UnityEngine.Rendering.HighDefinition.CustomPassVolume, Unity.RenderPipelines.HighDefinition.Runtime");
            var depthCopyPassType = System.Type.GetType("AutoBattlebot.Camera.HDRPDepthCopyPass, Assembly-CSharp");

            if (customPassVolumeType == null || depthCopyPassType == null)
            {
                throw new Exception("Required HDRP types not found");
            }

            // Create a game object for the custom pass volume
            var volumeGO = new GameObject("CameraSimulator_DepthCaptureVolume");
            volumeGO.transform.SetParent(transform);

            // Add CustomPassVolume component
            var volume = volumeGO.AddComponent(customPassVolumeType);

            // Set to global mode
            var modeProperty = customPassVolumeType.GetProperty("isGlobal");
            if (modeProperty != null)
            {
                modeProperty.SetValue(volume, true);
            }

            // Add the depth copy pass
            var addPassMethod = customPassVolumeType.GetMethod("AddPassOfType");
            if (addPassMethod != null)
            {
                var genericMethod = addPassMethod.MakeGenericMethod(depthCopyPassType);
                var pass = genericMethod.Invoke(volume, null);

                // Set the target texture on the pass
                if (pass != null)
                {
                    var targetField = depthCopyPassType.GetField("targetDepthTexture");
                    if (targetField != null)
                    {
                        targetField.SetValue(pass, _depthTarget);
                    }
                }
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
            if (_useLowLatencyMode)
            {
                CaptureFrameSynchronous();
            }
            else
            {
                CaptureFrameAsync();
            }
        }

        /// <summary>
        /// Low-latency synchronous capture. Blocks main thread but reduces latency from ~200ms to ~50ms.
        /// </summary>
        private void CaptureFrameSynchronous()
        {
            // Render the camera
            _camera.Render();

            // Handle depth capture (must happen before we change active render texture)
            if (_captureDepthThisFrame && _depthTarget != null && !_useCommandBufferDepth)
            {
                var depthTexture = Shader.GetGlobalTexture("_CameraDepthTexture");
                if (depthTexture != null)
                {
                    Graphics.Blit(depthTexture, _depthTarget);
                }
                else if (_depthCopyMaterial != null)
                {
                    Graphics.Blit(null, _depthTarget, _depthCopyMaterial);
                }
            }

            // Synchronous color readback using ReadPixels
            RenderTexture prevRT = RenderTexture.active;

            // Read color
            RenderTexture.active = _colorTarget;
            if (_syncColorTexture == null)
            {
                _syncColorTexture = new Texture2D(_imageWidth, _imageHeight, TextureFormat.RGBA32, false);
            }
            _syncColorTexture.ReadPixels(new Rect(0, 0, _imageWidth, _imageHeight), 0, 0, false);
            var colorData = _syncColorTexture.GetRawTextureData<byte>();
            ConvertRgbaToBgr(colorData, _bgrBuffer);

            // Read depth only if requested
            if (_captureDepthThisFrame && _depthTarget != null)
            {
                RenderTexture.active = _depthTarget;
                if (_syncDepthTexture == null)
                {
                    _syncDepthTexture = new Texture2D(_imageWidth, _imageHeight, TextureFormat.RFloat, false);
                }
                _syncDepthTexture.ReadPixels(new Rect(0, 0, _imageWidth, _imageHeight), 0, 0, false);
                var depthData = _syncDepthTexture.GetRawTextureData<float>();
                ConvertDepthToMetric(depthData, _depthBuffer);
            }

            RenderTexture.active = prevRT;

            // Write immediately (no waiting for callbacks)
            _colorReady = true;
            _depthReady = true;
            TryWriteFrame();
        }

        /// <summary>
        /// Standard async capture. Lower CPU usage but ~200ms latency due to GPU pipeline.
        /// </summary>
        private void CaptureFrameAsync()
        {
            // Set up readback flags before render
            _colorReadbackInFlight = true;
            _colorReady = false;

            // Only set up depth capture if requested
            if (_captureDepthThisFrame && _depthTarget != null)
            {
                _depthReadbackInFlight = true;
                _depthReady = false;
            }
            else
            {
                _depthReadbackInFlight = false;
                _depthReady = true;  // Mark as ready (empty)
            }

            // Render the camera
            _camera.Render();

            // Request color readback
            AsyncGPUReadback.Request(_colorTarget, 0, TextureFormat.RGBA32, OnColorReadbackComplete);

            // Handle depth capture only if requested
            if (_captureDepthThisFrame && _depthTarget != null)
            {
                // If command buffer is set up, it already copied depth during render
                // Otherwise, try to copy _CameraDepthTexture now
                if (!_useCommandBufferDepth)
                {
                    var depthTexture = Shader.GetGlobalTexture("_CameraDepthTexture");
                    if (depthTexture != null)
                    {
                        Graphics.Blit(depthTexture, _depthTarget);
                    }
                    else if (_depthCopyMaterial != null)
                    {
                        // Try using the depth copy material with current render state
                        Graphics.Blit(null, _depthTarget, _depthCopyMaterial);
                    }
                }

                AsyncGPUReadback.Request(_depthTarget, 0, TextureFormat.RFloat, OnDepthReadbackComplete);
            }
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
                FillFallbackDepth();
                _depthReady = true;
                TryWriteFrame();
                return;
            }

            var data = request.GetData<float>();

            // Debug: Check if depth data contains valid values
            float minDepth = float.MaxValue;
            float maxDepth = float.MinValue;
            int validCount = 0;
            for (int i = 0; i < Math.Min(1000, data.Length); i++)
            {
                float v = data[i];
                if (v > 0f && v < 1f)
                {
                    validCount++;
                    minDepth = Math.Min(minDepth, v);
                    maxDepth = Math.Max(maxDepth, v);
                }
            }

            if (validCount == 0)
            {
                Debug.LogWarning($"[CameraSimulator] Depth data appears empty (all 0 or 1). First values: {data[0]}, {data[1]}, {data[2]}");
            }
            else if (Time.frameCount % 150 == 0) // Log every ~5 seconds at 30fps
            {
                Debug.Log($"[CameraSimulator] Depth stats: min={minDepth:F4}, max={maxDepth:F4}, validSamples={validCount}/1000");
            }

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
                // Pass depth flag to communication bridge so it can signal C++ appropriately
                _communicationBridge.WriteFrame(outputBuffer, _depthBuffer, ConvertMatrixToRowMajor(pose), _captureDepthThisFrame);
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
                        metricDepth[dstRow + x] = _maxDepth;
                    }
                    else
                    {
                        float invNear = 1f / _nearClip;
                        float invFar = 1f / _farClip;
                        float linearDepth = 1f / (rawValue * (invNear - invFar) + invFar);

                        // Clamp to configured min/max depth range
                        metricDepth[dstRow + x] = Math.Clamp(linearDepth, _minDepth, _maxDepth);
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
            RenderPipelineManager.endContextRendering -= OnEndContextRendering;

            // Remove and dispose command buffer
            if (_depthCommandBuffer != null)
            {
                if (_camera != null)
                {
                    _camera.RemoveCommandBuffer(CameraEvent.AfterDepthTexture, _depthCommandBuffer);
                }
                _depthCommandBuffer.Dispose();
                _depthCommandBuffer = null;
            }

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
            _useCommandBufferDepth = false;

            // Clean up synchronous readback textures
            if (_syncColorTexture != null)
            {
                Destroy(_syncColorTexture);
                _syncColorTexture = null;
            }
            if (_syncDepthTexture != null)
            {
                Destroy(_syncDepthTexture);
                _syncDepthTexture = null;
            }
        }

        private void OnValidate()
        {
            if (_imageWidth < 1) _imageWidth = 1;
            if (_imageHeight < 1) _imageHeight = 1;
            if (_minDepth < 0.01f) _minDepth = 0.01f;
            if (_maxDepth < _minDepth) _maxDepth = _minDepth + 0.1f;
            if (_camera == null)
            {
                _camera = GetComponent<UnityCamera>();
            }
        }

        #endregion
    }
}
