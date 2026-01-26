// Auto-Battlebot Simulation System
// Display and performance settings for the simulation

using UnityEngine;
using AutoBattlebot.Core;

namespace AutoBattlebot.Core
{
    /// <summary>
    /// Configures display resolution, window mode, and target frame rate.
    /// Applies settings on initialization (before other components).
    /// </summary>
    [DefaultExecutionOrder(-200)]
    public class DisplaySettings : MonoBehaviour, IInitializable
    {
        #region Serialized Fields

        [Header("Resolution")]
        [SerializeField]
        [Tooltip("Screen width in pixels")]
        private int _screenWidth = 1920;

        [SerializeField]
        [Tooltip("Screen height in pixels")]
        private int _screenHeight = 1080;

        [SerializeField]
        [Tooltip("Window mode")]
        private FullScreenMode _windowMode = FullScreenMode.Windowed;

        [Header("Performance")]
        [SerializeField]
        [Tooltip("Target frame rate (-1 for unlimited, 0 for VSync)")]
        private int _targetFrameRate = 60;

        [SerializeField]
        [Tooltip("VSync count (0 = off, 1 = every VBlank, 2 = every other VBlank)")]
        private int _vSyncCount = 0;

        [Header("Quality")]
        [SerializeField]
        [Tooltip("Quality level index (-1 to keep current)")]
        private int _qualityLevel = -1;

        [Header("Background Behavior")]
        [SerializeField]
        [Tooltip("Run at full performance even when the window is not focused")]
        private bool _runInBackground = true;

        [Header("Debug")]
        [SerializeField]
        [Tooltip("Log settings on startup")]
        private bool _logSettings = true;

        #endregion

        #region Properties

        public int ScreenWidth => _screenWidth;
        public int ScreenHeight => _screenHeight;
        public FullScreenMode WindowMode => _windowMode;
        public int TargetFrameRate => _targetFrameRate;

        #endregion

        #region IInitializable Implementation

        public InitializationPhase Phase => InitializationPhase.PreInit;

        public void Initialize()
        {
            ApplySettings();
        }

        public void Shutdown()
        {
            // Nothing to clean up
        }

        #endregion

        #region Unity Lifecycle

        private void Awake()
        {
            // Apply settings immediately in Awake for fastest application
            // Will also be called via Initialize() for consistency with SimulationManager
            ApplySettings();
        }

        private void OnValidate()
        {
            // Ensure valid values
            if (_screenWidth < 640) _screenWidth = 640;
            if (_screenHeight < 480) _screenHeight = 480;
            if (_targetFrameRate < -1) _targetFrameRate = -1;
            if (_vSyncCount < 0) _vSyncCount = 0;
            if (_vSyncCount > 4) _vSyncCount = 4;
        }

        #endregion

        #region Public Methods

        /// <summary>
        /// Apply all display settings.
        /// </summary>
        public void ApplySettings()
        {
            // Set resolution and window mode
            Screen.SetResolution(_screenWidth, _screenHeight, _windowMode);

            // Set VSync
            QualitySettings.vSyncCount = _vSyncCount;

            // Set target frame rate (only applies when VSync is off)
            Application.targetFrameRate = _targetFrameRate;

            // Set quality level if specified
            if (_qualityLevel >= 0 && _qualityLevel < QualitySettings.names.Length)
            {
                QualitySettings.SetQualityLevel(_qualityLevel, true);
            }

            // Run in background (full performance when window not focused)
            Application.runInBackground = _runInBackground;

            if (_logSettings)
            {
                LogCurrentSettings();
            }
        }

        /// <summary>
        /// Set resolution at runtime.
        /// </summary>
        public void SetResolution(int width, int height, FullScreenMode mode)
        {
            _screenWidth = width;
            _screenHeight = height;
            _windowMode = mode;
            Screen.SetResolution(width, height, mode);
            
            if (_logSettings)
            {
                Debug.Log($"[DisplaySettings] Resolution changed to {width}x{height} ({mode})");
            }
        }

        /// <summary>
        /// Set target frame rate at runtime.
        /// </summary>
        public void SetTargetFrameRate(int fps)
        {
            _targetFrameRate = fps;
            Application.targetFrameRate = fps;
            
            if (_logSettings)
            {
                Debug.Log($"[DisplaySettings] Target frame rate set to {(fps == -1 ? "unlimited" : fps.ToString())}");
            }
        }

        #endregion

        #region Private Methods

        private void LogCurrentSettings()
        {
            string vsyncStr = _vSyncCount == 0 ? "Off" : $"Every {_vSyncCount} VBlank(s)";
            string fpsStr = _targetFrameRate == -1 ? "Unlimited" : 
                           (_targetFrameRate == 0 ? "VSync" : $"{_targetFrameRate} FPS");
            
            Debug.Log($"[DisplaySettings] Applied settings:\n" +
                     $"  Resolution: {_screenWidth}x{_screenHeight}\n" +
                     $"  Window Mode: {_windowMode}\n" +
                     $"  VSync: {vsyncStr}\n" +
                     $"  Target FPS: {fpsStr}\n" +
                     $"  Quality Level: {QualitySettings.names[QualitySettings.GetQualityLevel()]}\n" +
                     $"  Run In Background: {_runInBackground}");
        }

        #endregion
    }
}
