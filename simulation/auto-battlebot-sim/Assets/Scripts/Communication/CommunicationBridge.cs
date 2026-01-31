// Auto-Battlebot Simulation System
// CommunicationBridge - MonoBehaviour that orchestrates TCP communication with C++ application
//
// Responsibilities:
// - Manages TcpBridge lifecycle
// - Uses AsyncGPUReadback to capture RGB and depth textures
// - Sends frame data over TCP when client is connected
// - Receives and dispatches velocity commands
// - Integrates with SimulationManager initialization system

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using AutoBattlebot.Core;
using AutoBattlebot.Camera;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// MonoBehaviour that orchestrates communication between Unity and the C++ application.
    /// 
    /// This component:
    /// - Creates and manages the TcpBridge instance
    /// - Captures RGB and depth frames using AsyncGPUReadback
    /// - Sends frame data over TCP (with image data for Vulkan/fallback mode)
    /// - Receives velocity commands and fires events for robot control
    /// - Implements IInitializable for SimulationManager integration
    /// 
    /// Usage:
    /// 1. Add to a GameObject in the scene
    /// 2. Assign references to CameraSimulator and LinearDepthCapturePass
    /// 3. The bridge will auto-initialize with SimulationManager
    /// </summary>
    [DefaultExecutionOrder(-500)]
    public class CommunicationBridge : MonoBehaviour, IInitializable
    {
        #region Serialized Fields

        [Header("TCP Configuration")]
        [SerializeField]
        [Tooltip("TCP port for the bridge")]
        private int _port = TcpBridge.DEFAULT_PORT;

        [SerializeField]
        [Tooltip("Timeout for blocking operations (ms)")]
        private int _timeoutMs = TcpBridge.DEFAULT_TIMEOUT_MS;

        [Header("Camera References")]
        [SerializeField]
        [Tooltip("Reference to the RGB camera simulator")]
        private CameraSimulator _cameraSimulator;

        [SerializeField]
        [Tooltip("Reference to the depth capture pass (optional, via Custom Pass Volume)")]
        private LinearDepthCapturePass _depthCapturePass;

        [SerializeField]
        [Tooltip("Reference to the camera intrinsics provider")]
        private CameraIntrinsicsProvider _intrinsicsProvider;

        [Header("Frame Capture")]
        [SerializeField]
        [Tooltip("Enable depth capture (set false for RGB-only mode)")]
        private bool _captureDepth = true;

        [SerializeField]
        [Tooltip("Maximum number of pending AsyncGPUReadback requests")]
        private int _maxPendingReadbacks = 2;

        [Header("Debug")]
        [SerializeField]
        [Tooltip("Enable verbose logging")]
        private bool _verboseLogging = false;

        [SerializeField]
        [Tooltip("Log performance stats every N frames (0 to disable)")]
        private int _statsLogInterval = 300;

        #endregion

        #region Private Fields

        private TcpBridge _tcpBridge;
        private bool _isInitialized = false;
        private ulong _frameId = 0;

        // AsyncGPUReadback state
        private Queue<AsyncGPUReadbackRequest> _pendingRgbReadbacks = new Queue<AsyncGPUReadbackRequest>();
        private Queue<AsyncGPUReadbackRequest> _pendingDepthReadbacks = new Queue<AsyncGPUReadbackRequest>();
        private byte[] _rgbBuffer;
        private byte[] _depthBuffer;
        private bool _rgbReady = false;
        private bool _depthReady = false;
        private ulong _pendingFrameId = 0;
        private double _pendingTimestamp = 0;
        private Matrix4x4 _pendingPose = Matrix4x4.identity;

        // Performance tracking
        private int _framesSinceLastLog = 0;
        private float _lastStatsTime = 0;

        #endregion

        #region Properties

        /// <summary>
        /// The underlying TcpBridge instance.
        /// </summary>
        public TcpBridge Bridge => _tcpBridge;

        /// <summary>
        /// Whether a C++ client is connected.
        /// </summary>
        public bool IsClientConnected => _tcpBridge?.IsConnected ?? false;

        /// <summary>
        /// Current frame ID.
        /// </summary>
        public ulong CurrentFrameId => _frameId;

        #endregion

        #region Events

        /// <summary>
        /// Fired when a velocity command is received from the C++ application.
        /// </summary>
        public event Action<VelocityCommand> OnVelocityCommandReceived;

        /// <summary>
        /// Fired when the C++ client connects.
        /// </summary>
        public event Action OnClientConnected;

        /// <summary>
        /// Fired when the C++ client disconnects.
        /// </summary>
        public event Action OnClientDisconnected;

        #endregion

        #region IInitializable Implementation

        /// <summary>
        /// Initialization phase - runs during Init phase.
        /// </summary>
        public InitializationPhase Phase => InitializationPhase.Init;

        /// <summary>
        /// Initialize the communication bridge.
        /// </summary>
        public void Initialize()
        {
            if (_isInitialized)
            {
                Log("Already initialized");
                return;
            }

            // Auto-find references if not assigned
            FindReferences();

            // Create TcpBridge
            _tcpBridge = new TcpBridge(_port, _timeoutMs);

            // Subscribe to events
            _tcpBridge.OnClientConnected += HandleClientConnected;
            _tcpBridge.OnClientDisconnected += HandleClientDisconnected;
            _tcpBridge.OnCommandReceived += HandleCommandReceived;

            // Start listening
            if (!_tcpBridge.Initialize())
            {
                Debug.LogError("[CommunicationBridge] Failed to initialize TcpBridge");
                return;
            }

            // Allocate buffers based on camera resolution
            AllocateBuffers();

            _isInitialized = true;
            Log($"Initialized on port {_port}");
        }

        /// <summary>
        /// Shutdown the communication bridge.
        /// </summary>
        public void Shutdown()
        {
            if (!_isInitialized)
            {
                return;
            }

            if (_tcpBridge != null)
            {
                _tcpBridge.OnClientConnected -= HandleClientConnected;
                _tcpBridge.OnClientDisconnected -= HandleClientDisconnected;
                _tcpBridge.OnCommandReceived -= HandleCommandReceived;
                _tcpBridge.Dispose();
                _tcpBridge = null;
            }

            _isInitialized = false;
            Log("Shutdown complete");
        }

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            // Register with SimulationManager if it exists
            if (SimulationManager.Exists)
            {
                SimulationManager.Instance.RegisterInitializable(this);
            }
        }

        private void Start()
        {
            // If SimulationManager didn't initialize us, do it ourselves
            if (!_isInitialized)
            {
                Initialize();
            }
        }

        private void Update()
        {
            if (!_isInitialized || _tcpBridge == null)
            {
                return;
            }

            // Poll for commands
            if (_tcpBridge.TryReceiveCommand(out var command, out bool isNew))
            {
                if (isNew && _verboseLogging)
                {
                    Log($"Command: linear=({command.LinearX:F2}, {command.LinearY:F2}), angular={command.AngularZ:F2}");
                }
            }

            // Process completed readbacks
            ProcessPendingReadbacks();

            // Log stats periodically
            if (_statsLogInterval > 0)
            {
                _framesSinceLastLog++;
                if (_framesSinceLastLog >= _statsLogInterval)
                {
                    LogPerformanceStats();
                    _framesSinceLastLog = 0;
                }
            }
        }

        private void LateUpdate()
        {
            if (!_isInitialized || !IsClientConnected)
            {
                return;
            }

            // Check if we can start a new capture
            if (_pendingRgbReadbacks.Count < _maxPendingReadbacks)
            {
                CaptureFrame();
            }
        }

        private void OnDestroy()
        {
            Shutdown();
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Manually trigger a frame capture and send.
        /// </summary>
        public void CaptureAndSendFrame()
        {
            if (!_isInitialized || !IsClientConnected)
            {
                return;
            }

            CaptureFrame();
        }

        /// <summary>
        /// Get the current camera pose (world-to-camera transform).
        /// </summary>
        public Matrix4x4 GetCurrentPose()
        {
            if (_cameraSimulator != null && _cameraSimulator.Camera != null)
            {
                // Get camera's world-to-local matrix (view matrix)
                return _cameraSimulator.Camera.worldToCameraMatrix;
            }
            return Matrix4x4.identity;
        }

        #endregion

        #region Private Methods

        private void FindReferences()
        {
            if (_cameraSimulator == null)
            {
                _cameraSimulator = FindFirstObjectByType<CameraSimulator>();
                if (_cameraSimulator == null)
                {
                    Debug.LogWarning("[CommunicationBridge] No CameraSimulator found in scene");
                }
            }

            if (_intrinsicsProvider == null && _cameraSimulator != null)
            {
                _intrinsicsProvider = _cameraSimulator.GetComponent<CameraIntrinsicsProvider>();
            }

            if (_intrinsicsProvider == null)
            {
                _intrinsicsProvider = FindFirstObjectByType<CameraIntrinsicsProvider>();
            }

            // Find depth capture pass in Custom Pass Volumes
            if (_depthCapturePass == null && _captureDepth)
            {
                var volumes = FindObjectsByType<UnityEngine.Rendering.HighDefinition.CustomPassVolume>(FindObjectsSortMode.None);
                foreach (var volume in volumes)
                {
                    foreach (var pass in volume.customPasses)
                    {
                        if (pass is LinearDepthCapturePass depthPass)
                        {
                            _depthCapturePass = depthPass;
                            break;
                        }
                    }
                    if (_depthCapturePass != null) break;
                }

                if (_depthCapturePass == null)
                {
                    Debug.LogWarning("[CommunicationBridge] No LinearDepthCapturePass found - depth capture disabled");
                    _captureDepth = false;
                }
            }
        }

        private void AllocateBuffers()
        {
            int width = _intrinsicsProvider?.Width ?? 1280;
            int height = _intrinsicsProvider?.Height ?? 720;

            // RGBA format: 4 bytes per pixel
            int rgbSize = width * height * 4;
            _rgbBuffer = new byte[rgbSize];

            if (_captureDepth)
            {
                // R32F format: 4 bytes per pixel (float)
                int depthSize = width * height * 4;
                _depthBuffer = new byte[depthSize];
            }

            Log($"Allocated buffers: RGB={rgbSize / 1024}KB, Depth={(_captureDepth ? _depthBuffer.Length / 1024 : 0)}KB");
        }

        private void CaptureFrame()
        {
            if (_cameraSimulator == null || !_cameraSimulator.HasTexture)
            {
                return;
            }

            _frameId++;
            _pendingFrameId = _frameId;
            _pendingTimestamp = Time.timeAsDouble;
            _pendingPose = GetCurrentPose();

            // Request RGB readback
            var rgbRequest = AsyncGPUReadback.Request(_cameraSimulator.RgbTexture, 0, TextureFormat.RGBA32);
            _pendingRgbReadbacks.Enqueue(rgbRequest);
            _rgbReady = false;

            // Request depth readback if enabled
            if (_captureDepth && _depthCapturePass != null && _depthCapturePass.HasTexture)
            {
                var depthRequest = AsyncGPUReadback.Request(_depthCapturePass.DepthTexture, 0, TextureFormat.RFloat);
                _pendingDepthReadbacks.Enqueue(depthRequest);
                _depthReady = false;
            }
            else
            {
                _depthReady = true; // No depth to wait for
            }
        }

        private void ProcessPendingReadbacks()
        {
            // Process RGB readbacks
            while (_pendingRgbReadbacks.Count > 0)
            {
                var request = _pendingRgbReadbacks.Peek();

                if (request.hasError)
                {
                    Debug.LogError("[CommunicationBridge] RGB readback error");
                    _pendingRgbReadbacks.Dequeue();
                    continue;
                }

                if (!request.done)
                {
                    break; // Still waiting
                }

                // Copy data to buffer
                var data = request.GetData<byte>();
                if (data.Length <= _rgbBuffer.Length)
                {
                    data.CopyTo(_rgbBuffer);
                    _rgbReady = true;
                }
                else
                {
                    Debug.LogWarning($"[CommunicationBridge] RGB data size mismatch: {data.Length} vs {_rgbBuffer.Length}");
                }

                _pendingRgbReadbacks.Dequeue();
                break; // Process one at a time
            }

            // Process depth readbacks
            while (_pendingDepthReadbacks.Count > 0)
            {
                var request = _pendingDepthReadbacks.Peek();

                if (request.hasError)
                {
                    Debug.LogError("[CommunicationBridge] Depth readback error");
                    _pendingDepthReadbacks.Dequeue();
                    _depthReady = true; // Skip depth for this frame
                    continue;
                }

                if (!request.done)
                {
                    break; // Still waiting
                }

                // Copy data to buffer
                var data = request.GetData<byte>();
                if (_depthBuffer != null && data.Length <= _depthBuffer.Length)
                {
                    data.CopyTo(_depthBuffer);
                    _depthReady = true;
                }

                _pendingDepthReadbacks.Dequeue();
                break; // Process one at a time
            }

            // Send frame when both RGB and depth are ready
            if (_rgbReady && _depthReady && IsClientConnected)
            {
                SendFrame();
                _rgbReady = false;
                _depthReady = false;
            }
        }

        private void SendFrame()
        {
            if (_tcpBridge == null || !IsClientConnected)
            {
                return;
            }

            int width = _intrinsicsProvider?.Width ?? 1280;
            int height = _intrinsicsProvider?.Height ?? 720;

            bool success;

            if (_captureDepth && _depthBuffer != null)
            {
                success = _tcpBridge.SendFrameReadyWithData(
                    _pendingFrameId,
                    _pendingTimestamp,
                    _pendingPose,
                    width, height, _rgbBuffer,
                    width, height, _depthBuffer);
            }
            else
            {
                success = _tcpBridge.SendFrameReadyWithData(
                    _pendingFrameId,
                    _pendingTimestamp,
                    _pendingPose,
                    width, height, _rgbBuffer,
                    0, 0, null);
            }

            if (_verboseLogging && success)
            {
                Log($"Sent frame {_pendingFrameId}");
            }
        }

        private void HandleClientConnected()
        {
            Log("C++ client connected");

            // Send camera intrinsics
            if (_intrinsicsProvider != null)
            {
                _tcpBridge.SendIntrinsics(_intrinsicsProvider.Intrinsics);
            }

            OnClientConnected?.Invoke();
        }

        private void HandleClientDisconnected()
        {
            Log("C++ client disconnected");

            // Clear pending readbacks
            _pendingRgbReadbacks.Clear();
            _pendingDepthReadbacks.Clear();
            _rgbReady = false;
            _depthReady = false;

            OnClientDisconnected?.Invoke();
        }

        private void HandleCommandReceived(VelocityCommand command)
        {
            OnVelocityCommandReceived?.Invoke(command);
        }

        private void LogPerformanceStats()
        {
            if (_tcpBridge == null)
            {
                return;
            }

            float elapsed = Time.time - _lastStatsTime;
            float fps = _framesSinceLastLog / elapsed;

            Debug.Log($"[CommunicationBridge] Stats: {fps:F1} fps, " +
                      $"Frame={_frameId}, " +
                      $"Connected={IsClientConnected}, " +
                      $"Pending RGB={_pendingRgbReadbacks.Count}, " +
                      $"Pending Depth={_pendingDepthReadbacks.Count}\n" +
                      _tcpBridge.GetPerformanceReport());

            _lastStatsTime = Time.time;
        }

        private void Log(string message)
        {
            if (_verboseLogging)
            {
                Debug.Log($"[CommunicationBridge] {message}");
            }
        }

        #endregion

        #region Editor Support

#if UNITY_EDITOR
        [ContextMenu("Log Status")]
        private void EditorLogStatus()
        {
            Debug.Log($"[CommunicationBridge] Status:\n" +
                      $"  Initialized: {_isInitialized}\n" +
                      $"  Connected: {IsClientConnected}\n" +
                      $"  Frame ID: {_frameId}\n" +
                      $"  Camera: {(_cameraSimulator != null ? "Found" : "Missing")}\n" +
                      $"  Depth Pass: {(_depthCapturePass != null ? "Found" : "Missing")}\n" +
                      $"  Intrinsics: {(_intrinsicsProvider != null ? "Found" : "Missing")}");
        }

        [ContextMenu("Force Initialize")]
        private void EditorForceInitialize()
        {
            if (_isInitialized)
            {
                Shutdown();
            }
            Initialize();
        }
#endif

        #endregion
    }
}
