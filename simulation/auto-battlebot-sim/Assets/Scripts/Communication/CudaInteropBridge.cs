// Auto-Battlebot Simulation System
// CudaInteropBridge - Coordinates CUDA Interop with TCP communication
//
// This MonoBehaviour manages bidirectional communication with the C++ application:
// - GPU textures (RGB/Depth) via CUDA Interop (zero-copy)
// - Small data (pose, velocity commands, intrinsics) via TCP socket
//
// Script Execution Order: -100 (runs before most other scripts)

using System;
using UnityEngine;
using UnityEngine.Rendering;
using AutoBattlebot.Core;
using AutoBattlebot.NativePlugins;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Bridge component that coordinates CUDA Interop for GPU texture sharing
    /// with TCP socket communication for small data exchange.
    /// 
    /// Responsibilities:
    /// - Register RenderTextures with CUDA native plugin
    /// - Manage TCP connection with C++ application
    /// - Send frame-ready signals with pose data via TCP
    /// - Receive velocity commands via TCP
    /// - Send camera intrinsics on connection
    /// 
    /// Usage:
    /// 1. Attach to a GameObject in the scene
    /// 2. Assign RGB and Depth RenderTextures
    /// 3. Assign a CameraIntrinsicsProvider (or leave null for default ZED 2i intrinsics)
    /// 4. Call SignalFrameReady() after each render
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class CudaInteropBridge : MonoBehaviour, IInitializable
    {
        #region Serialized Fields

        [Header("CUDA Interop Settings")]
        [SerializeField]
        [Tooltip("CUDA device ID to use (typically 0)")]
        private int _cudaDeviceId = 0;

        [SerializeField]
        [Tooltip("RGB RenderTexture to share with C++ via CUDA")]
        private RenderTexture _rgbTexture;

        [SerializeField]
        [Tooltip("Depth RenderTexture to share with C++ via CUDA")]
        private RenderTexture _depthTexture;

        [Header("TCP Communication Settings")]
        [SerializeField]
        [Tooltip("TCP port for small data exchange")]
        private int _tcpPort = TcpBridge.DEFAULT_PORT;

        [SerializeField]
        [Tooltip("Timeout for TCP operations in milliseconds")]
        private int _tcpTimeoutMs = TcpBridge.DEFAULT_TIMEOUT_MS;

        [Header("Camera Intrinsics")]
        [SerializeField]
        [Tooltip("Camera intrinsics provider (required)")]
        private CameraIntrinsicsProvider _intrinsicsProvider;

        [Header("Frame Synchronization")]
        [SerializeField]
        [Tooltip("Use lockstep synchronization (wait for C++ acknowledgment)")]
        private bool _useLockstep = false;

        [SerializeField]
        [Tooltip("Timeout for lockstep wait in milliseconds")]
        private int _lockstepTimeoutMs = 100;

        [Header("Diagnostics")]
        [SerializeField]
        [Tooltip("Log performance metrics periodically")]
        private bool _logPerformanceMetrics = false;

        [SerializeField]
        [Tooltip("Interval in seconds between performance logs")]
        private float _metricsLogInterval = 5f;

        [Header("Recovery")]
        [SerializeField]
        [Tooltip("Automatically reinitialize on error")]
        private bool _autoRecover = true;

        [SerializeField]
        [Tooltip("Seconds between recovery attempts")]
        private float _recoverIntervalSeconds = 1f;

        #endregion

        #region Private Fields

        private TcpBridge _tcpBridge;
        private bool _isActive = false;
        private bool _cudaInitialized = false;
        private bool _texturesRegistered = false;

        private ulong _frameId = 0;
        private VelocityCommand _currentCommand = VelocityCommand.Zero;
        private CameraIntrinsics _intrinsics;

        private float _lastMetricsLogTime = 0f;
        private float _nextRecoverTime = 0f;

        private CommandBuffer _renderCommandBuffer;

        #endregion

        #region Properties

        /// <summary>
        /// Whether the bridge is fully active and ready for use.
        /// </summary>
        public bool IsActive => _isActive;

        /// <summary>
        /// Whether CUDA Interop is initialized.
        /// </summary>
        public bool IsCudaInitialized => _cudaInitialized;

        /// <summary>
        /// Whether textures are registered with CUDA.
        /// </summary>
        public bool TexturesRegistered => _texturesRegistered;

        /// <summary>
        /// Whether the TCP connection to C++ is established.
        /// </summary>
        public bool IsTcpConnected => _tcpBridge?.IsConnected ?? false;

        /// <summary>
        /// Current frame ID.
        /// </summary>
        public ulong FrameId => _frameId;

        /// <summary>
        /// Most recent velocity command received from C++.
        /// </summary>
        public VelocityCommand CurrentCommand => _currentCommand;

        /// <summary>
        /// The TCP bridge instance.
        /// </summary>
        public TcpBridge TcpBridge => _tcpBridge;

        /// <summary>
        /// Camera intrinsics being used.
        /// </summary>
        public CameraIntrinsics Intrinsics => _intrinsics;

        /// <summary>
        /// The camera intrinsics provider (if assigned).
        /// </summary>
        public CameraIntrinsicsProvider IntrinsicsProvider
        {
            get => _intrinsicsProvider;
            set
            {
                _intrinsicsProvider = value;
                if (value != null)
                {
                    UpdateIntrinsicsFromProvider();
                }
            }
        }

        #endregion

        #region Events

        /// <summary>
        /// Fired when the C++ application connects.
        /// </summary>
        public event Action OnClientConnected;

        /// <summary>
        /// Fired when the C++ application disconnects.
        /// </summary>
        public event Action OnClientDisconnected;

        /// <summary>
        /// Fired when a new velocity command is received.
        /// </summary>
        public event Action<VelocityCommand> OnCommandReceived;

        /// <summary>
        /// Fired when C++ requests a frame.
        /// </summary>
        public event Action<bool> OnFrameRequested;

        #endregion

        #region IInitializable Implementation

        public InitializationPhase Phase => InitializationPhase.Init;

        public void Initialize()
        {
            // Only initialize in HIL mode
            if (SimulationManager.Instance != null &&
                SimulationManager.Instance.CurrentMode != RobotSimulationMode.HardwareInLoop)
            {
                Debug.Log("[CudaInteropBridge] Skipping initialization (not in HIL mode)");
                return;
            }

            // Validate required references
            if (_intrinsicsProvider == null)
            {
                Debug.LogError("[CudaInteropBridge] CameraIntrinsicsProvider is required but not assigned");
                return;
            }

            Debug.Log("[CudaInteropBridge] Initializing...");

            bool cudaOk = InitializeCuda();
            bool tcpOk = InitializeTcp();

            if (cudaOk && tcpOk)
            {
                // Register textures if available
                if (_rgbTexture != null && _depthTexture != null)
                {
                    RegisterTextures(_rgbTexture, _depthTexture);
                }

                _isActive = true;
                Debug.Log("[CudaInteropBridge] Initialization complete");
            }
            else
            {
                Debug.LogError("[CudaInteropBridge] Initialization failed");
            }
        }

        public void Shutdown()
        {
            Debug.Log("[CudaInteropBridge] Shutting down...");

            if (_logPerformanceMetrics)
            {
                LogPerformanceMetrics();
            }

            // Cleanup render command buffer
            if (_renderCommandBuffer != null)
            {
                _renderCommandBuffer.Release();
                _renderCommandBuffer = null;
            }

            // Shutdown TCP
            _tcpBridge?.Dispose();
            _tcpBridge = null;

            // Shutdown CUDA
            if (_cudaInitialized)
            {
                CudaInterop.Shutdown();
                _cudaInitialized = false;
            }

            _texturesRegistered = false;
            _isActive = false;
        }

        #endregion

        #region Unity Lifecycle

        private void Start()
        {
            // Auto-initialize if SimulationManager is not being used
            if (SimulationManager.Instance == null && !_isActive)
            {
                Initialize();
            }
        }

        private void Update()
        {
            if (!_isActive)
            {
                // Try to recover if auto-recovery is enabled
                if (_autoRecover && Time.realtimeSinceStartup >= _nextRecoverTime)
                {
                    _nextRecoverTime = Time.realtimeSinceStartup + _recoverIntervalSeconds;
                    TryRecover();
                }
                return;
            }

            // Poll for incoming commands
            if (_tcpBridge != null && _tcpBridge.IsConnected)
            {
                if (_tcpBridge.TryReceiveCommand(out var command, out bool isNew))
                {
                    if (isNew)
                    {
                        _currentCommand = command;
                        OnCommandReceived?.Invoke(command);
                    }
                }
            }

            // Log performance metrics periodically
            if (_logPerformanceMetrics && Time.time - _lastMetricsLogTime >= _metricsLogInterval)
            {
                LogPerformanceMetrics();
                _lastMetricsLogTime = Time.time;
            }
        }

        private void OnDestroy()
        {
            Shutdown();
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Registers RGB and Depth RenderTextures with CUDA for zero-copy access.
        /// </summary>
        /// <param name="rgb">RGB RenderTexture.</param>
        /// <param name="depth">Depth RenderTexture.</param>
        /// <returns>True if registration succeeded.</returns>
        public bool RegisterTextures(RenderTexture rgb, RenderTexture depth)
        {
            if (!_cudaInitialized)
            {
                Debug.LogError("[CudaInteropBridge] CUDA not initialized");
                return false;
            }

            _rgbTexture = rgb;
            _depthTexture = depth;

            bool rgbOk = CudaInterop.RegisterTexture(rgb, CudaInteropTextureType.RGB);
            bool depthOk = CudaInterop.RegisterTexture(depth, CudaInteropTextureType.Depth);

            if (rgbOk && depthOk)
            {
                _texturesRegistered = true;

                // Update intrinsics from provider
                if (!UpdateIntrinsicsFromProvider())
                {
                    Debug.LogError("[CudaInteropBridge] Failed to get valid intrinsics");
                    return false;
                }

                // Send intrinsics if connected
                if (_tcpBridge != null && _tcpBridge.IsConnected && !_tcpBridge.IntrinsicsSent)
                {
                    _tcpBridge.SendIntrinsics(_intrinsics);
                }

                Debug.Log($"[CudaInteropBridge] Textures registered: {rgb.width}x{rgb.height}");
                return true;
            }
            else
            {
                Debug.LogError("[CudaInteropBridge] Failed to register textures");
                return false;
            }
        }

        /// <summary>
        /// Signals that a frame is ready for processing by C++.
        /// This triggers CUDA sync and sends pose data via TCP.
        /// </summary>
        /// <param name="cameraPose">Camera pose matrix.</param>
        /// <param name="hasDepth">Whether the frame includes depth data.</param>
        /// <returns>True if signal was sent successfully.</returns>
        public bool SignalFrameReady(Matrix4x4 cameraPose, bool hasDepth = true)
        {
            if (!_isActive)
            {
                return false;
            }

            _frameId++;

            // Signal CUDA sync (inserts GL fence)
            if (_texturesRegistered)
            {
                CudaInterop.SyncAndNotify();
            }

            // Send frame data via TCP
            if (_tcpBridge != null && _tcpBridge.IsConnected)
            {
                bool sent = _tcpBridge.SendFrameReady(_frameId, Time.timeAsDouble, cameraPose, hasDepth);

                if (_useLockstep && sent)
                {
                    // Wait for command response
                    if (_tcpBridge.WaitForCommand(_lockstepTimeoutMs, out var command))
                    {
                        _currentCommand = command;
                        OnCommandReceived?.Invoke(command);
                    }
                }

                return sent;
            }

            return true;  // No TCP connection, but CUDA sync happened
        }

        /// <summary>
        /// Signals frame ready and issues CUDA sync via command buffer (render thread safe).
        /// </summary>
        /// <param name="commandBuffer">Command buffer to append sync event to.</param>
        /// <param name="cameraPose">Camera pose matrix.</param>
        /// <param name="hasDepth">Whether the frame includes depth data.</param>
        public void SignalFrameReadyDeferred(CommandBuffer commandBuffer, Matrix4x4 cameraPose, bool hasDepth = true)
        {
            if (!_isActive)
            {
                return;
            }

            _frameId++;

            // Issue CUDA sync via command buffer (render thread safe)
            if (_texturesRegistered)
            {
                CudaInterop.IssueRenderEvent(commandBuffer);
            }

            // Send frame data via TCP (happens on main thread, but that's OK for small data)
            if (_tcpBridge != null && _tcpBridge.IsConnected)
            {
                _tcpBridge.SendFrameReady(_frameId, Time.timeAsDouble, cameraPose, hasDepth);
            }
        }

        /// <summary>
        /// Manually sends camera intrinsics to the C++ application.
        /// </summary>
        /// <param name="intrinsics">Camera intrinsics to send.</param>
        /// <returns>True if sent successfully.</returns>
        public bool SendIntrinsics(CameraIntrinsics intrinsics)
        {
            _intrinsics = intrinsics;

            if (_tcpBridge != null && _tcpBridge.IsConnected)
            {
                return _tcpBridge.SendIntrinsics(intrinsics);
            }

            return false;
        }

        /// <summary>
        /// Gets the most recent velocity command received.
        /// </summary>
        /// <param name="command">Output: the command.</param>
        /// <param name="isNew">Output: true if this is a new command since last call.</param>
        /// <returns>True if a command is available.</returns>
        public bool TryGetCommand(out VelocityCommand command, out bool isNew)
        {
            command = _currentCommand;
            isNew = false;

            if (_tcpBridge != null && _tcpBridge.IsConnected)
            {
                return _tcpBridge.TryReceiveCommand(out command, out isNew);
            }

            return false;
        }

        /// <summary>
        /// Checks if the TCP connection is alive.
        /// </summary>
        /// <param name="timeoutMs">Ping timeout in milliseconds.</param>
        /// <returns>True if connection is alive.</returns>
        public bool Ping(int timeoutMs = 100)
        {
            return _tcpBridge?.Ping(timeoutMs) ?? false;
        }

        /// <summary>
        /// Gets a combined performance report.
        /// </summary>
        public string GetPerformanceReport()
        {
            string report = "[CudaInteropBridge] Performance Report:\n";
            report += $"  Active: {_isActive}\n";
            report += $"  CUDA Initialized: {_cudaInitialized}\n";
            report += $"  Textures Registered: {_texturesRegistered}\n";
            report += $"  TCP Connected: {IsTcpConnected}\n";
            report += $"  Frame ID: {_frameId}\n";

            if (_cudaInitialized)
            {
                report += CudaInterop.GetPerformanceReport() + "\n";
            }

            if (_tcpBridge != null)
            {
                report += _tcpBridge.GetPerformanceReport();
            }

            return report;
        }

        #endregion

        #region Private Methods

        private bool InitializeCuda()
        {
            if (_cudaInitialized)
            {
                return true;
            }

            if (!CudaInterop.IsPluginAvailable)
            {
                Debug.LogWarning("[CudaInteropBridge] CUDA Interop plugin not available");
                return false;
            }

            if (CudaInterop.Initialize(_cudaDeviceId))
            {
                _cudaInitialized = true;
                Debug.Log($"[CudaInteropBridge] CUDA initialized on device {_cudaDeviceId}");
                return true;
            }
            else
            {
                Debug.LogError("[CudaInteropBridge] Failed to initialize CUDA");
                return false;
            }
        }

        private bool InitializeTcp()
        {
            if (_tcpBridge != null && _tcpBridge.IsListening)
            {
                return true;
            }

            _tcpBridge = new TcpBridge(_tcpPort, _tcpTimeoutMs);

            // Subscribe to events
            _tcpBridge.OnClientConnected += HandleClientConnected;
            _tcpBridge.OnClientDisconnected += HandleClientDisconnected;
            _tcpBridge.OnCommandReceived += HandleCommandReceived;
            _tcpBridge.OnFrameRequested += HandleFrameRequested;

            if (_tcpBridge.Initialize())
            {
                Debug.Log($"[CudaInteropBridge] TCP server started on port {_tcpPort}");

                // Prepare intrinsics (may fail if provider not assigned yet)
                if (!UpdateIntrinsicsFromProvider())
                {
                    Debug.LogWarning("[CudaInteropBridge] Intrinsics not available yet - will be set when textures are registered");
                }

                return true;
            }
            else
            {
                Debug.LogError("[CudaInteropBridge] Failed to initialize TCP server");
                return false;
            }
        }

        private bool UpdateIntrinsicsFromProvider()
        {
            if (_intrinsicsProvider == null)
            {
                Debug.LogError("[CudaInteropBridge] CameraIntrinsicsProvider is required but not assigned");
                return false;
            }

            _intrinsics = _intrinsicsProvider.Intrinsics;

            if (!_intrinsics.IsValid())
            {
                Debug.LogError("[CudaInteropBridge] CameraIntrinsicsProvider returned invalid intrinsics");
                return false;
            }

            return true;
        }

        private void TryRecover()
        {
            Debug.Log("[CudaInteropBridge] Attempting recovery...");

            bool recovered = true;

            if (!_cudaInitialized)
            {
                recovered &= InitializeCuda();
            }

            if (_tcpBridge == null || !_tcpBridge.IsListening)
            {
                _tcpBridge?.Dispose();
                _tcpBridge = null;
                recovered &= InitializeTcp();
            }

            if (_cudaInitialized && !_texturesRegistered && _rgbTexture != null && _depthTexture != null)
            {
                recovered &= RegisterTextures(_rgbTexture, _depthTexture);
            }

            if (recovered)
            {
                _isActive = true;
                Debug.Log("[CudaInteropBridge] Recovery successful");
            }
        }

        private void HandleClientConnected()
        {
            Debug.Log("[CudaInteropBridge] C++ application connected");

            // Send intrinsics on connection
            if (_intrinsics.IsValid())
            {
                _tcpBridge.SendIntrinsics(_intrinsics);
            }

            OnClientConnected?.Invoke();
        }

        private void HandleClientDisconnected()
        {
            Debug.LogWarning("[CudaInteropBridge] C++ application disconnected");
            OnClientDisconnected?.Invoke();
        }

        private void HandleCommandReceived(VelocityCommand command)
        {
            _currentCommand = command;
            OnCommandReceived?.Invoke(command);
        }

        private void HandleFrameRequested(bool withDepth)
        {
            OnFrameRequested?.Invoke(withDepth);
        }

        private void LogPerformanceMetrics()
        {
            Debug.Log(GetPerformanceReport());
        }

        #endregion
    }
}
