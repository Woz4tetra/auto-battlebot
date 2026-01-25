// Auto-Battlebot Simulation System
// Virtual camera simulator matching ZED 2i characteristics

using UnityEngine;
using AutoBattlebot.Core;

namespace AutoBattlebot.Camera
{
    /// <summary>
    /// Simulates the Stereolabs ZED 2i camera, providing RGB images,
    /// depth images, and pose tracking.
    /// 
    /// Script Execution Order: 0 (default)
    /// </summary>
    public class CameraSimulator : MonoBehaviour, IInitializable
    {
        #region Serialized Fields
        
        [Header("Camera Settings")]
        [SerializeField]
        [Tooltip("Width of the captured images")]
        private int _imageWidth = 1280;
        
        [SerializeField]
        [Tooltip("Height of the captured images")]
        private int _imageHeight = 720;
        
        [SerializeField]
        [Tooltip("Target capture frame rate")]
        private int _captureFrameRate = 30;
        
        #endregion

        #region Properties
        
        public int ImageWidth => _imageWidth;
        public int ImageHeight => _imageHeight;
        public int CaptureFrameRate => _captureFrameRate;
        
        #endregion

        #region IInitializable Implementation
        
        public InitializationPhase Phase => InitializationPhase.Init;
        
        public void Initialize()
        {
            Debug.Log($"[CameraSimulator] Initializing camera ({_imageWidth}x{_imageHeight} @ {_captureFrameRate}fps)...");
            // TODO: Set up render textures and camera configuration
            Debug.Log("[CameraSimulator] Camera ready");
        }
        
        public void Shutdown()
        {
            Debug.Log("[CameraSimulator] Releasing camera resources...");
            // TODO: Clean up render textures
        }
        
        #endregion
    }
}
