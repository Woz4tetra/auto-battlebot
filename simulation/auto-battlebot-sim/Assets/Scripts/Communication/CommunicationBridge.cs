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
using AutoBattlebot.SimulatedCamera;

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
        [Tooltip("Reference to the camera intrinsics provider")]
        private CameraIntrinsicsProvider _intrinsicsProvider;

        [Header("Frame Capture")]
        [SerializeField]
        [Tooltip("Enable depth capture (set false for RGB-only mode)")]
        private bool _captureDepth = true;

        [Header("Debug")]
        [SerializeField]
        [Tooltip("Enable verbose logging")]
        private bool _verboseLogging = false;

        [SerializeField]
        [Tooltip("Log performance stats every N frames (0 to disable)")]
        private int _statsLogFrameInterval = 300;

        #endregion

        #region Private Fields

        private TcpBridge _tcpBridge;
        private bool _isInitialized = false;
        private ulong _frameId = 0;
        private LinearDepthCapturePass _depthCapturePass;

        // ==============================================
        // CAPTURE STATE MACHINE
        // ==============================================
        // Simple state machine for frame capture:
        //   Idle -> Capturing -> ReadyToSend -> Idle
        //
        // Flow each frame (all in LateUpdate):
        //   1. Poll for commands (depth requests)
        //   2. If Idle: start new capture
        //   3. If Capturing: check if readbacks complete
        //   4. If ReadyToSend: send frame, go to Idle
        // ==============================================

        private enum CaptureState
        {
            Idle,           // Ready to start new capture
            Capturing,      // Waiting for GPU readback(s)
            ReadyToSend     // Readbacks done, ready to send
        }

        private CaptureState _state = CaptureState.Idle;
        private bool _depthRequestPending = false;     // C++ wants depth on next capture
        private bool _currentCaptureHasDepth = false;  // Current capture includes depth

        // Current capture's readback requests (only one capture in flight at a time)
        private AsyncGPUReadbackRequest? _rgbReadback;
        private AsyncGPUReadbackRequest? _depthReadback;

        // Buffers for frame data
        private byte[] _rgbBuffer;
        private byte[] _depthBuffer;

        // Current capture metadata
        private ulong _pendingFrameId = 0;
        private double _pendingTimestamp = 0;
        private Matrix4x4 _pendingPose = Matrix4x4.identity;

        // Performance tracking
        private int _framesSinceLastLog = 0;
        private float _lastStatsTime = 0;

        // Startup delay to ensure camera has rendered
        private int _startupFrameDelay = 3;

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
        /// Initialization phase - runs during PostInit phase.
        /// This ensures CameraSimulator and SimulationCameraController have initialized first.
        /// </summary>
        public InitializationPhase Phase => InitializationPhase.PostInit;

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
            _tcpBridge.OnFrameRequested += HandleFrameRequested;

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
                _tcpBridge.OnFrameRequested -= HandleFrameRequested;
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

        /// <summary>
        /// Main update loop - all frame processing happens here in sequence.
        /// </summary>
        private void LateUpdate()
        {
            if (!_isInitialized || _tcpBridge == null)
            {
                return;
            }

            // ============================================
            // STEP 1: Poll for commands (depth requests)
            // ============================================
            PollCommands();

            // ============================================
            // STEP 2: Log stats periodically
            // ============================================
            if (_statsLogFrameInterval > 0)
            {
                _framesSinceLastLog++;
                if (_framesSinceLastLog >= _statsLogFrameInterval)
                {
                    LogPerformanceStats();
                    _framesSinceLastLog = 0;
                }
            }

            // Need client connected to capture/send
            if (!IsClientConnected)
            {
                return;
            }

            // Wait for startup frames to let connection stabilize
            // PollCommands was already called above, so request_frame can be received
            if (_startupFrameDelay > 0)
            {
                _startupFrameDelay--;
                if (_startupFrameDelay == 0)
                {
                    Debug.Log($"[CommunicationBridge] Startup delay complete, DepthPending={_depthRequestPending}");
                }
                return;
            }

            // ============================================
            // STEP 3: Process state machine
            // Chain transitions within same frame for lower latency
            // ============================================
            ProcessStateMachine();
        }

        private void PollCommands()
        {
            if (_tcpBridge.TryReceiveCommand(out var command, out bool isNew))
            {
                if (isNew && _verboseLogging)
                {
                    Log($"Command: linear=({command.LinearX:F2}, {command.LinearY:F2}), angular={command.AngularZ:F2}");
                }
            }
        }

        /// <summary>
        /// Process the capture state machine, chaining transitions within same frame.
        /// </summary>
        private void ProcessStateMachine()
        {
            // Allow multiple transitions per frame for lower latency
            // e.g., Idle -> Capturing -> (next frame) -> ReadyToSend -> Idle
            // or if readback is instant: Idle -> Capturing -> ReadyToSend -> Idle (same frame)
            
            int maxIterations = 3;  // Prevent infinite loops
            for (int i = 0; i < maxIterations; i++)
            {
                var prevState = _state;
                
                switch (_state)
                {
                    case CaptureState.Idle:
                        StartCapture();
                        break;

                    case CaptureState.Capturing:
                        CheckReadbacksComplete();
                        break;

                    case CaptureState.ReadyToSend:
                        SendFrameAndReset();
                        break;
                }

                // Stop if state didn't change (waiting for something)
                if (_state == prevState)
                {
                    break;
                }
            }
        }

        private void OnDestroy()
        {
            Shutdown();
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Manually trigger a frame capture (will be sent when readback completes).
        /// Note: This is handled automatically by the state machine in LateUpdate.
        /// </summary>
        public void CaptureAndSendFrame()
        {
            if (!_isInitialized || !IsClientConnected)
            {
                return;
            }

            // Force start a new capture if idle
            if (_state == CaptureState.Idle)
            {
                StartCapture();
            }
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
        }

        /// <summary>
        /// Attempts to find the depth capture pass. Can be called multiple times
        /// as the pass may be created later by SimulationCameraController.
        /// </summary>
        private bool TryFindDepthPass()
        {
            if (_depthCapturePass != null)
            {
                Debug.Log($"[CommunicationBridge] Depth capture is already found");
                return true;  // Already found
            }

            if (!_captureDepth)
            {
                Debug.Log($"[CommunicationBridge] Depth capture is disabled");
                return false;  // Depth capture disabled by user
            }

            // Try to find via SimulationCameraController first
            var cameraController = _cameraSimulator?.GetComponent<SimulationCameraController>();
            if (cameraController == null)
            {
                cameraController = FindFirstObjectByType<SimulationCameraController>();
            }

            if (cameraController != null && cameraController.DepthPass != null)
            {
                _depthCapturePass = cameraController.DepthPass;
            }
            else
            {
                // Fallback: search Custom Pass Volumes directly
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
            }

            if (_depthCapturePass != null)
            {
                Debug.Log($"[CommunicationBridge] Found depth capture: {_depthCapturePass.width}x{_depthCapturePass.height}");
                return true;
            }

            Debug.LogWarning($"[CommunicationBridge] Failed to find depth capture");
            return false;
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

        // ============================================
        // STATE MACHINE METHODS
        // ============================================

        /// <summary>
        /// Start a new frame capture (RGB always, depth if requested).
        /// Transitions: Idle -> Capturing
        /// </summary>
        private void StartCapture()
        {
            if (_cameraSimulator == null || !_cameraSimulator.HasTexture)
            {
                return;
            }

            var rgbTexture = _cameraSimulator.RgbTexture;
            if (rgbTexture == null || !rgbTexture.IsCreated())
            {
                return;
            }

            // Find depth pass on first capture if needed
            if (_captureDepth && _depthCapturePass == null)
            {
                TryFindDepthPass();
            }

            // Consume pending depth request
            _currentCaptureHasDepth = _captureDepth && _depthRequestPending && 
                                       _depthCapturePass != null && _depthCapturePass.HasTexture;
            if (_currentCaptureHasDepth)
            {
                _depthRequestPending = false;  // Consumed
            }

            // Store frame metadata
            _frameId++;
            _pendingFrameId = _frameId;
            _pendingTimestamp = Time.timeAsDouble;
            _pendingPose = GetCurrentPose();

            // Log first capture
            if (_pendingFrameId == 1)
            {
                Debug.Log($"[CommunicationBridge] First capture: " +
                    $"RGB={rgbTexture.width}x{rgbTexture.height} ({rgbTexture.graphicsFormat}), " +
                    $"Depth={(_depthCapturePass != null ? "available" : "not available")}");
            }

            // Request RGB readback
            _rgbReadback = AsyncGPUReadback.Request(rgbTexture);

            // Request depth readback if needed
            if (_currentCaptureHasDepth)
            {
                var depthTexture = _depthCapturePass.DepthTexture;
                _depthReadback = AsyncGPUReadback.Request(depthTexture);
                
                if (_verboseLogging)
                {
                    Debug.Log($"[CommunicationBridge] Capturing frame {_pendingFrameId} with depth");
                }
            }
            else
            {
                _depthReadback = null;
            }

            _state = CaptureState.Capturing;
        }

        /// <summary>
        /// Check if all pending readbacks are complete.
        /// Transitions: Capturing -> ReadyToSend (when done)
        /// </summary>
        private void CheckReadbacksComplete()
        {
            // Check RGB readback
            if (_rgbReadback.HasValue)
            {
                var request = _rgbReadback.Value;
                
                if (request.hasError)
                {
                    Debug.LogError($"[CommunicationBridge] RGB readback error for frame {_pendingFrameId}");
                    ResetToIdle();
                    return;
                }

                if (!request.done)
                {
                    return;  // Still waiting
                }

                // Copy RGB data
                var data = request.GetData<byte>();
                if (_rgbBuffer == null || data.Length != _rgbBuffer.Length)
                {
                    _rgbBuffer = new byte[data.Length];
                }
                data.CopyTo(_rgbBuffer);
                _rgbReadback = null;  // Mark as processed
            }

            // Check depth readback (if we're capturing depth)
            if (_currentCaptureHasDepth && _depthReadback.HasValue)
            {
                var request = _depthReadback.Value;
                
                if (request.hasError)
                {
                    Debug.LogError($"[CommunicationBridge] Depth readback error for frame {_pendingFrameId}");
                    _currentCaptureHasDepth = false;  // Send without depth
                    _depthReadback = null;
                }
                else if (!request.done)
                {
                    return;  // Still waiting
                }
                else
                {
                    // Copy depth data
                    var data = request.GetData<byte>();
                    if (_depthBuffer == null || data.Length != _depthBuffer.Length)
                    {
                        _depthBuffer = new byte[data.Length];
                    }
                    data.CopyTo(_depthBuffer);
                    _depthReadback = null;  // Mark as processed
                }
            }

            // Both readbacks complete
            _state = CaptureState.ReadyToSend;
        }

        /// <summary>
        /// Send the captured frame and reset state.
        /// Transitions: ReadyToSend -> Idle
        /// </summary>
        private void SendFrameAndReset()
        {
            SendFrame();
            ResetToIdle();
        }

        private void ResetToIdle()
        {
            _state = CaptureState.Idle;
            _rgbReadback = null;
            _depthReadback = null;
            _currentCaptureHasDepth = false;
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
            bool sendDepth = _currentCaptureHasDepth && _depthBuffer != null && _depthBuffer.Length > 0;

            // Log frame details
            if (_pendingFrameId <= 3 || _verboseLogging)
            {
                Debug.Log($"[CommunicationBridge] SendFrame {_pendingFrameId}: " +
                    $"RGB={_rgbBuffer?.Length ?? 0} bytes, " +
                    $"Depth={(sendDepth ? _depthBuffer.Length.ToString() + " bytes" : "none")}, " +
                    $"Dims={width}x{height}");
            }

            if (sendDepth)
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

            if (!success)
            {
                Debug.LogWarning($"[CommunicationBridge] Failed to send frame {_pendingFrameId}");
            }
        }

        private void HandleClientConnected()
        {
            Debug.Log("[CommunicationBridge] C++ client connected");

            // Send camera intrinsics FIRST before any other state changes
            // This ensures C++ can process intrinsics and send request_frame
            // before we start polling or sending frames
            if (_intrinsicsProvider != null)
            {
                _tcpBridge.SendIntrinsics(_intrinsicsProvider.Intrinsics);
            }

            // Reset all state
            // Add a longer startup delay to allow for full round-trip:
            // Unity sends intrinsics -> C++ receives -> C++ sends request_frame -> Unity receives
            // At 120 FPS, each frame is ~8ms. Network round-trip could be 10-50ms.
            // Use 10 frames (~80ms) to be safe.
            _startupFrameDelay = 10;
            _state = CaptureState.Idle;
            _depthRequestPending = false;
            _currentCaptureHasDepth = false;
            _rgbReadback = null;
            _depthReadback = null;

            OnClientConnected?.Invoke();
        }

        private void HandleClientDisconnected()
        {
            Debug.Log("[CommunicationBridge] C++ client disconnected");

            // Reset all state
            _state = CaptureState.Idle;
            _depthRequestPending = false;
            _currentCaptureHasDepth = false;
            _rgbReadback = null;
            _depthReadback = null;

            OnClientDisconnected?.Invoke();
        }

        private void HandleCommandReceived(VelocityCommand command)
        {
            OnVelocityCommandReceived?.Invoke(command);
        }

        private void HandleFrameRequested(bool withDepth)
        {
            // C++ is requesting depth on the next capture
            Debug.Log($"[CommunicationBridge] HandleFrameRequested called: withDepth={withDepth}");
            if (withDepth)
            {
                _depthRequestPending = true;
                Debug.Log("[CommunicationBridge] Depth requested for next capture");
            }
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
                      $"State={_state}, " +
                      $"Connected={IsClientConnected}, " +
                      $"DepthPending={_depthRequestPending}\n" +
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
                      $"  State: {_state}\n" +
                      $"  Frame ID: {_frameId}\n" +
                      $"  Depth Request Pending: {_depthRequestPending}\n" +
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
