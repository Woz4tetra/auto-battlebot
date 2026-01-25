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
    /// Script Execution Order: -100
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class CommunicationBridge : MonoBehaviour, IInitializable
    {
        #region Serialized Fields
        
        [Header("Shared Memory Settings")]
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
        private bool _isActive = false;
        private float _lastMetricsLogTime = 0f;
        
        #endregion

        #region Properties
        
        /// <summary>
        /// The shared memory writer for frame data.
        /// </summary>
        public SharedMemoryWriter FrameWriter => _frameWriter;
        
        /// <summary>
        /// Whether the communication bridge is active and ready.
        /// </summary>
        public bool IsActive => _isActive;
        
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
            
            // Create and initialize the frame writer
            _frameWriter = new SharedMemoryWriter(
                _imageWidth,
                _imageHeight,
                _frameMemoryName,
                _useDoubleBuffering);
            
            if (_frameWriter.Initialize())
            {
                _isActive = true;
                Debug.Log("[CommunicationBridge] Communication bridge ready");
            }
            else
            {
                Debug.LogError("[CommunicationBridge] Failed to initialize shared memory writer");
            }
        }
        
        public void Shutdown()
        {
            Debug.Log("[CommunicationBridge] Closing communication channels...");
            
            if (_frameWriter != null)
            {
                if (_logPerformanceMetrics)
                {
                    Debug.Log(_frameWriter.GetPerformanceReport());
                }
                
                _frameWriter.Dispose();
                _frameWriter = null;
            }
            
            _isActive = false;
        }
        
        #endregion

        #region Unity Lifecycle
        
        private void Update()
        {
            // Log performance metrics periodically if enabled
            if (_logPerformanceMetrics && _isActive && _frameWriter != null)
            {
                if (Time.time - _lastMetricsLogTime >= _metricsLogInterval)
                {
                    Debug.Log(_frameWriter.GetPerformanceReport());
                    _lastMetricsLogTime = Time.time;
                }
            }
        }
        
        private void OnDestroy()
        {
            // Ensure cleanup if not already done
            if (_frameWriter != null)
            {
                _frameWriter.Dispose();
                _frameWriter = null;
            }
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
        
        #endregion
    }
}
