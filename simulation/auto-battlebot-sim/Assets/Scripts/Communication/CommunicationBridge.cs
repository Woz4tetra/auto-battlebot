// Auto-Battlebot Simulation System
// Communication bridge for IPC with C++ application

using UnityEngine;
using AutoBattlebot.Core;
using SimulationMode = AutoBattlebot.Core.SimulationMode;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Manages bidirectional communication with the C++ application
    /// via shared memory and synchronization sockets.
    /// 
    /// - Writes: RGB, depth, and pose data to C++ (via SharedMemoryWriter)
    /// - Reads: Velocity commands from C++ (via SharedMemoryReader)
    /// - Sync: Frame timing coordination (via SyncSocket)
    /// 
    /// Script Execution Order: -100
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class CommunicationBridge : MonoBehaviour, IInitializable
    {
        #region Serialized Fields

        [Header("Frame Writer Settings")]
        [SerializeField]
        [Tooltip("Name of the shared memory region for frame data")]
        private string _frameMemoryName = SharedMemoryWriter.DEFAULT_MEMORY_NAME;

        [SerializeField]
        [Tooltip("Image width for shared memory buffer")]
        private int _imageWidth = 1280;

        [SerializeField]
        [Tooltip("Image height for shared memory buffer")]
        private int _imageHeight = 720;

        [SerializeField]
        [Tooltip("Enable double-buffering to prevent tearing")]
        private bool _useDoubleBuffering = true;

        [Header("Command Reader Settings")]
        [SerializeField]
        [Tooltip("Name of the shared memory region for velocity commands")]
        private string _commandMemoryName = SharedMemoryReader.DEFAULT_MEMORY_NAME;

        [SerializeField]
        [Tooltip("Maximum allowed linear velocity in m/s")]
        private double _maxLinearVelocity = VelocityCommand.DEFAULT_MAX_LINEAR_VELOCITY;

        [SerializeField]
        [Tooltip("Maximum allowed angular velocity in rad/s")]
        private double _maxAngularVelocity = VelocityCommand.DEFAULT_MAX_ANGULAR_VELOCITY;

        [Header("Synchronization Settings")]
        [SerializeField]
        [Tooltip("Enable synchronization socket for frame timing")]
        private bool _enableSyncSocket = true;

        [SerializeField]
        [Tooltip("Synchronization mode")]
        private SyncMode _syncMode = SyncMode.Lockstep;

        [SerializeField]
        [Tooltip("Socket path for Unix domain sockets (Linux)")]
        private string _socketPath = SyncSocket.DEFAULT_SOCKET_PATH;

        [SerializeField]
        [Tooltip("TCP port for Windows fallback")]
        private int _tcpPort = SyncSocket.DEFAULT_TCP_PORT;

        [SerializeField]
        [Tooltip("Timeout for synchronization operations in milliseconds")]
        private int _syncTimeoutMs = SyncSocket.DEFAULT_TIMEOUT_MS;

        [Header("Diagnostics")]
        [SerializeField]
        [Tooltip("Log performance metrics periodically")]
        private bool _logPerformanceMetrics = false;

        [SerializeField]
        [Tooltip("Interval in seconds between performance logs")]
        private float _metricsLogInterval = 5f;

        [Header("Recovery")]
        [SerializeField]
        [Tooltip("Automatically reinitialize IPC if Unity or C++ restarts.")]
        private bool _autoRecover = true;

        [SerializeField]
        [Tooltip("Seconds between recovery attempts when a component is down.")]
        private float _recoverIntervalSeconds = 0.5f;

        #endregion

        #region Private Fields

        private SharedMemoryWriter _frameWriter;
        private SharedMemoryReader _commandReader;
        private SyncSocket _syncSocket;
        private bool _isActive = false;
        private float _lastMetricsLogTime = 0f;
        private float _nextRecoverTime = 0f;
        private VelocityCommand _currentCommand = VelocityCommand.Zero;
        private ulong _frameId = 0;

        // Frame request tracking (for request-driven capture)
        private bool _pendingFrameRequest = false;
        private bool _pendingDepthRequest = false;

        #endregion

        #region Properties

        /// <summary>
        /// The shared memory writer for frame data.
        /// </summary>
        public SharedMemoryWriter FrameWriter => _frameWriter;

        /// <summary>
        /// The shared memory reader for velocity commands.
        /// </summary>
        public SharedMemoryReader CommandReader => _commandReader;

        /// <summary>
        /// The synchronization socket for frame timing.
        /// </summary>
        public SyncSocket SyncSocket => _syncSocket;

        /// <summary>
        /// Whether the communication bridge is active and ready.
        /// </summary>
        public bool IsActive => _isActive;

        /// <summary>
        /// Whether the sync socket is connected to C++.
        /// </summary>
        public bool IsSyncConnected => _syncSocket?.IsConnected ?? false;

        /// <summary>
        /// The most recent valid velocity command received.
        /// </summary>
        public VelocityCommand CurrentCommand => _currentCommand;

        /// <summary>
        /// Current frame ID (incremented each WriteFrame call).
        /// </summary>
        public ulong FrameId => _frameId;

        /// <summary>
        /// Whether a frame request is pending from C++.
        /// </summary>
        public bool HasPendingFrameRequest => _pendingFrameRequest;

        /// <summary>
        /// Whether the pending frame request includes depth.
        /// </summary>
        public bool PendingRequestIncludesDepth => _pendingDepthRequest;

        #endregion

        #region Events

        /// <summary>
        /// Fired when a new velocity command is received from C++.
        /// </summary>
        public event System.Action<VelocityCommand> OnCommandReceived;

        /// <summary>
        /// Fired when C++ requests a frame. Parameter indicates whether depth is requested.
        /// </summary>
        public event System.Action<bool> OnFrameRequested;

        /// <summary>
        /// Fired when the sync socket connects.
        /// </summary>
        public event System.Action OnSyncConnected;

        /// <summary>
        /// Fired when the sync socket disconnects.
        /// </summary>
        public event System.Action OnSyncDisconnected;

        #endregion

        #region IInitializable Implementation

        public InitializationPhase Phase => InitializationPhase.Init;

        public void Initialize()
        {
            // Only initialize in HIL mode
            if (SimulationManager.Instance.CurrentMode != SimulationMode.HardwareInLoop)
            {
                Debug.Log("[CommunicationBridge] Skipping initialization (not in HIL mode)");
                return;
            }

            Debug.Log("[CommunicationBridge] Initializing communication...");

            bool writerOk = InitializeFrameWriter();
            bool readerOk = InitializeCommandReader();
            bool syncOk = InitializeSyncSocket();

            if (writerOk && readerOk && (_enableSyncSocket ? syncOk : true))
            {
                _isActive = true;
                Debug.Log("[CommunicationBridge] Communication bridge ready");
            }
            else
            {
                Debug.LogError("[CommunicationBridge] Failed to initialize communication bridge");
            }
        }

        public void Shutdown()
        {
            Debug.Log("[CommunicationBridge] Closing communication channels...");

            if (_logPerformanceMetrics)
            {
                LogAllPerformanceMetrics();
            }

            _syncSocket?.Dispose();
            _syncSocket = null;

            _frameWriter?.Dispose();
            _frameWriter = null;

            _commandReader?.Dispose();
            _commandReader = null;

            _isActive = false;
        }

        #endregion

        #region Unity Lifecycle

        private void Update()
        {
            // Auto-recover if we're in HIL mode but init failed (e.g., after restarts)
            if (!_isActive)
            {
                if (_autoRecover && SimulationManager.Instance.CurrentMode == SimulationMode.HardwareInLoop)
                {
                    float now = Time.realtimeSinceStartup;
                    if (now >= _nextRecoverTime)
                    {
                        _nextRecoverTime = now + Mathf.Max(0.1f, _recoverIntervalSeconds);
                        TryRecover();
                    }
                }
                return;
            }

            // Check for frame requests from C++ (request-driven capture mode)
            if (_enableSyncSocket && _syncSocket != null && _syncSocket.IsConnected)
            {
                if (_syncSocket.TryGetFrameRequest(out bool withDepth))
                {
                    _pendingFrameRequest = true;
                    _pendingDepthRequest = withDepth;
                    OnFrameRequested?.Invoke(withDepth);
                    
                    if (Time.frameCount % 100 == 0)
                    {
                        Debug.Log($"[CommunicationBridge] Received frame request: withDepth={withDepth}");
                    }
                }
            }

            // In free-running mode, poll for commands
            // In lockstep mode, commands are read after frame sync
            if (_syncMode == SyncMode.FreeRunning || !_enableSyncSocket)
            {
                PollCommands();
            }
            else
            {
                // In lockstep mode we depend on a live sync connection. If it drops, keep trying.
                if (_enableSyncSocket && (_syncSocket == null || !_syncSocket.IsConnected))
                {
                    _syncSocket?.TryReconnect();
                }
            }

            // Log performance metrics periodically if enabled
            if (_logPerformanceMetrics)
            {
                if (Time.time - _lastMetricsLogTime >= _metricsLogInterval)
                {
                    LogAllPerformanceMetrics();
                    _lastMetricsLogTime = Time.time;
                }
            }
        }

        private void OnDestroy()
        {
            // Ensure cleanup if not already done
            _syncSocket?.Dispose();
            _syncSocket = null;

            _frameWriter?.Dispose();
            _frameWriter = null;

            _commandReader?.Dispose();
            _commandReader = null;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Writes a frame to shared memory and signals C++ if sync is enabled.
        /// In lockstep mode, blocks until C++ responds with command-ready.
        /// </summary>
        /// <param name="rgbTexture">RGB texture to write.</param>
        /// <param name="depthTexture">Depth texture to write.</param>
        /// <param name="cameraPose">Camera pose matrix.</param>
        /// <returns>True if write (and sync if enabled) was successful.</returns>
        public bool WriteFrame(Texture2D rgbTexture, Texture2D depthTexture, Matrix4x4 cameraPose)
        {
            if (!_isActive || _frameWriter == null)
            {
                return false;
            }

            bool writeOk = _frameWriter.WriteFrame(rgbTexture, depthTexture, cameraPose, Time.timeAsDouble);
            if (!writeOk)
            {
                return false;
            }

            _frameId++;
            return SignalAndSync();
        }

        /// <summary>
        /// Writes a frame to shared memory using raw data arrays.
        /// In lockstep mode, blocks until C++ responds with command-ready.
        /// </summary>
        /// <param name="rgbData">BGR image data.</param>
        /// <param name="depthData">Depth data as float array.</param>
        /// <param name="poseMatrix">Pose matrix as double array.</param>
        /// <returns>True if write (and sync if enabled) was successful.</returns>
        public bool WriteFrame(byte[] rgbData, float[] depthData, double[] poseMatrix)
        {
            // Determine if this frame has depth (non-null and non-empty depth data)
            bool hasDepth = depthData != null && depthData.Length > 0;
            return WriteFrame(rgbData, depthData, poseMatrix, hasDepth);
        }

        /// <summary>
        /// Writes a frame to shared memory with explicit depth availability flag.
        /// </summary>
        /// <param name="rgbData">BGR image data.</param>
        /// <param name="depthData">Depth data as float array (can be null/empty if hasDepth is false).</param>
        /// <param name="poseMatrix">Pose matrix as double array.</param>
        /// <param name="hasDepth">Whether this frame includes valid depth data.</param>
        /// <returns>True if write (and sync if enabled) was successful.</returns>
        public bool WriteFrame(byte[] rgbData, float[] depthData, double[] poseMatrix, bool hasDepth)
        {
            if (!_isActive || _frameWriter == null)
            {
                return false;
            }

            // Pass hasDepth flag to writer to skip depth write when not available
            bool writeOk = _frameWriter.WriteFrame(rgbData, depthData, poseMatrix, Time.timeAsDouble, hasDepth);
            if (!writeOk)
            {
                return false;
            }

            _frameId++;
            
            // Clear pending request since we're fulfilling it
            _pendingFrameRequest = false;
            _pendingDepthRequest = false;
            
            return SignalAndSync(hasDepth);
        }

        private void TryRecover()
        {
            bool ok = true;

            // Recreate missing/broken components. Each init is expected to be idempotent/safe.
            if (_frameWriter == null || !_frameWriter.IsInitialized)
            {
                try { _frameWriter?.Dispose(); } catch { }
                _frameWriter = null;
                ok &= InitializeFrameWriter();
            }

            if (_commandReader == null || !_commandReader.IsInitialized)
            {
                try { _commandReader?.Dispose(); } catch { }
                _commandReader = null;
                ok &= InitializeCommandReader();
            }

            if (_enableSyncSocket)
            {
                // For server mode: "connected" is optional, but we must be listening.
                if (_syncSocket == null || !_syncSocket.IsListening)
                {
                    try { _syncSocket?.Dispose(); } catch { }
                    _syncSocket = null;
                    ok &= InitializeSyncSocket();
                }
            }

            if (ok)
            {
                _isActive = true;
                Debug.Log("[CommunicationBridge] Recovery succeeded");
            }
        }

        /// <summary>
        /// Signals that a frame is ready without writing shared memory.
        /// This is intended for GPU-share (native plugin) transport, where the image/depth live on the GPU.
        /// </summary>
        /// <param name="hasDepth">Whether the corresponding GPU-shared frame includes valid depth.</param>
        /// <returns>True if the signal (and lockstep sync if enabled) succeeded.</returns>
        public bool SignalFrameReadyWithoutWrite(bool hasDepth)
        {
            if (!_isActive)
            {
                return false;
            }

            _frameId++;

            // Clear pending request since we're fulfilling it
            _pendingFrameRequest = false;
            _pendingDepthRequest = false;

            return SignalAndSync(hasDepth);
        }

        /// <summary>
        /// Tries to read the latest velocity command.
        /// </summary>
        /// <param name="command">The command that was read.</param>
        /// <param name="isNewCommand">True if this is a new command (command_id changed).</param>
        /// <returns>True if read was successful.</returns>
        public bool TryReadCommand(out VelocityCommand command, out bool isNewCommand)
        {
            command = VelocityCommand.Zero;
            isNewCommand = false;

            if (!_isActive || _commandReader == null)
            {
                return false;
            }

            return _commandReader.TryReadCommand(out command, out isNewCommand);
        }

        /// <summary>
        /// Manually signals frame ready without writing new frame data.
        /// Useful for synchronization testing.
        /// </summary>
        /// <returns>True if signal was successful.</returns>
        public bool SignalFrameReady()
        {
            if (!_enableSyncSocket || _syncSocket == null)
            {
                return true;  // No sync required
            }

            return _syncSocket.SignalFrameReady(_frameId);
        }

        /// <summary>
        /// Attempts to reconnect the sync socket.
        /// </summary>
        /// <returns>True if reconnection was successful or not needed.</returns>
        public bool TryReconnectSync()
        {
            if (!_enableSyncSocket || _syncSocket == null)
            {
                return true;
            }

            return _syncSocket.TryReconnect();
        }

        #endregion

        #region Private Methods

        private bool InitializeFrameWriter()
        {
            _frameWriter = new SharedMemoryWriter(
                _imageWidth,
                _imageHeight,
                _frameMemoryName,
                _useDoubleBuffering);

            if (!_frameWriter.Initialize(deleteFileOnDispose: false))
            {
                Debug.LogError("[CommunicationBridge] Failed to initialize frame writer");
                return false;
            }

            return true;
        }

        private bool InitializeCommandReader()
        {
            _commandReader = new SharedMemoryReader(
                _commandMemoryName,
                _maxLinearVelocity,
                _maxAngularVelocity);

            // Create the shared memory if it doesn't exist (C++ may not be running yet)
            if (!_commandReader.Initialize(createIfNotExists: true, deleteFileOnDispose: false))
            {
                Debug.LogError("[CommunicationBridge] Failed to initialize command reader");
                return false;
            }

            return true;
        }

        private bool InitializeSyncSocket()
        {
            if (!_enableSyncSocket)
            {
                Debug.Log("[CommunicationBridge] Sync socket disabled");
                return true;
            }

            _syncSocket = new SyncSocket(
                isServer: true,
                mode: _syncMode,
                socketPath: _socketPath,
                tcpPort: _tcpPort,
                timeoutMs: _syncTimeoutMs);

            // Subscribe to events
            _syncSocket.OnConnected += HandleSyncConnected;
            _syncSocket.OnDisconnected += HandleSyncDisconnected;
            _syncSocket.OnCommandReady += HandleCommandReady;

            if (!_syncSocket.Initialize())
            {
                Debug.LogError("[CommunicationBridge] Failed to initialize sync socket");
                return false;
            }

            Debug.Log($"[CommunicationBridge] Sync socket initialized (mode: {_syncMode})");
            return true;
        }

        private bool SignalAndSync(bool hasDepth = true)
        {
            if (!_enableSyncSocket || _syncSocket == null)
            {
                return true;  // No sync required
            }

            if (!_syncSocket.IsConnected)
            {
                // Try to recover the connection before failing.
                _syncSocket.TryReconnect();

                // In free-running mode, continue even without sync
                if (_syncMode == SyncMode.FreeRunning)
                {
                    return true;
                }
                // In lockstep mode, we can't proceed without connection
                Debug.LogWarning("[CommunicationBridge] Sync socket not connected, cannot signal frame ready");
                return false;
            }

            // Use the overload that specifies depth availability
            bool signalOk = _syncSocket.SignalFrameReady(hasDepth, _frameId);

            // In lockstep mode, command was received during SignalFrameReady
            if (signalOk && _syncMode == SyncMode.Lockstep)
            {
                PollCommands();
            }

            return signalOk;
        }

        private void PollCommands()
        {
            if (_commandReader == null)
            {
                return;
            }

            if (_commandReader.TryReadCommand(out var command, out bool isNew))
            {
                if (isNew)
                {
                    _currentCommand = command;
                    OnCommandReceived?.Invoke(command);
                }
            }
        }

        private void HandleSyncConnected()
        {
            Debug.Log("[CommunicationBridge] C++ application connected");
            OnSyncConnected?.Invoke();
        }

        private void HandleSyncDisconnected()
        {
            Debug.LogWarning("[CommunicationBridge] C++ application disconnected");
            OnSyncDisconnected?.Invoke();
        }

        private void HandleCommandReady()
        {
            // In lockstep mode, this is called after each frame sync
            // Commands are polled immediately
        }

        private void LogAllPerformanceMetrics()
        {
            if (_frameWriter != null)
            {
                Debug.Log(_frameWriter.GetPerformanceReport());
            }
            if (_commandReader != null)
            {
                Debug.Log(_commandReader.GetPerformanceReport());
            }
            if (_syncSocket != null)
            {
                Debug.Log(_syncSocket.GetPerformanceReport());
            }
        }

        #endregion
    }
}
