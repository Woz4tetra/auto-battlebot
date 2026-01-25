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

        [Header("Diagnostics")]
        [SerializeField]
        [Tooltip("Log performance metrics periodically")]
        private bool _logPerformanceMetrics = false;

        [SerializeField]
        [Tooltip("Interval in seconds between performance logs")]
        private float _metricsLogInterval = 5f;

        #endregion

        #region Private Fields

        private SharedMemoryWriter _frameWriter;
        private SharedMemoryReader _commandReader;
        private bool _isActive = false;
        private float _lastMetricsLogTime = 0f;
        private VelocityCommand _currentCommand = VelocityCommand.Zero;

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
        /// Whether the communication bridge is active and ready.
        /// </summary>
        public bool IsActive => _isActive;

        /// <summary>
        /// The most recent valid velocity command received.
        /// </summary>
        public VelocityCommand CurrentCommand => _currentCommand;

        #endregion

        #region Events

        /// <summary>
        /// Fired when a new velocity command is received from C++.
        /// </summary>
        public event System.Action<VelocityCommand> OnCommandReceived;

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

            Debug.Log("[CommunicationBridge] Initializing shared memory communication...");

            bool writerOk = InitializeFrameWriter();
            bool readerOk = InitializeCommandReader();

            if (writerOk && readerOk)
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
                if (_frameWriter != null)
                {
                    Debug.Log(_frameWriter.GetPerformanceReport());
                }
                if (_commandReader != null)
                {
                    Debug.Log(_commandReader.GetPerformanceReport());
                }
            }

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
            if (!_isActive)
            {
                return;
            }

            // Poll for new commands
            PollCommands();

            // Log performance metrics periodically if enabled
            if (_logPerformanceMetrics)
            {
                if (Time.time - _lastMetricsLogTime >= _metricsLogInterval)
                {
                    if (_frameWriter != null)
                    {
                        Debug.Log(_frameWriter.GetPerformanceReport());
                    }
                    if (_commandReader != null)
                    {
                        Debug.Log(_commandReader.GetPerformanceReport());
                    }
                    _lastMetricsLogTime = Time.time;
                }
            }
        }

        private void OnDestroy()
        {
            // Ensure cleanup if not already done
            _frameWriter?.Dispose();
            _frameWriter = null;

            _commandReader?.Dispose();
            _commandReader = null;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Writes a frame to shared memory.
        /// </summary>
        /// <param name="rgbTexture">RGB texture to write.</param>
        /// <param name="depthTexture">Depth texture to write.</param>
        /// <param name="cameraPose">Camera pose matrix.</param>
        /// <returns>True if write was successful.</returns>
        public bool WriteFrame(Texture2D rgbTexture, Texture2D depthTexture, Matrix4x4 cameraPose)
        {
            if (!_isActive || _frameWriter == null)
            {
                return false;
            }

            return _frameWriter.WriteFrame(rgbTexture, depthTexture, cameraPose, Time.timeAsDouble);
        }

        /// <summary>
        /// Writes a frame to shared memory using raw data arrays.
        /// </summary>
        /// <param name="rgbData">BGR image data.</param>
        /// <param name="depthData">Depth data as float array.</param>
        /// <param name="poseMatrix">Pose matrix as double array.</param>
        /// <returns>True if write was successful.</returns>
        public bool WriteFrame(byte[] rgbData, float[] depthData, double[] poseMatrix)
        {
            if (!_isActive || _frameWriter == null)
            {
                return false;
            }

            return _frameWriter.WriteFrame(rgbData, depthData, poseMatrix, Time.timeAsDouble);
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

        #endregion

        #region Private Methods

        private bool InitializeFrameWriter()
        {
            _frameWriter = new SharedMemoryWriter(
                _imageWidth,
                _imageHeight,
                _frameMemoryName,
                _useDoubleBuffering);

            if (!_frameWriter.Initialize())
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
            if (!_commandReader.Initialize(createIfNotExists: true))
            {
                Debug.LogError("[CommunicationBridge] Failed to initialize command reader");
                return false;
            }

            return true;
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

        #endregion
    }
}
