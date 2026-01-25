// Auto-Battlebot Simulation System
// Data generation controller for synthetic training data

using UnityEngine;
using AutoBattlebot.Core;
using SimulationMode = AutoBattlebot.Core.SimulationMode;

namespace AutoBattlebot.DataGeneration
{
    /// <summary>
    /// Controls synthetic data generation for training ML models.
    /// Receives commands from Python orchestrator via TCP.
    /// 
    /// Script Execution Order: 0 (default)
    /// </summary>
    public class DataGenerationController : MonoBehaviour, IInitializable
    {
        #region Serialized Fields
        
        [Header("Server Settings")]
        [SerializeField]
        [Tooltip("TCP port for Python orchestrator connection")]
        private int _serverPort = 9999;
        
        [SerializeField]
        [Tooltip("Whether to start the server automatically")]
        private bool _autoStartServer = false;
        
        #endregion

        #region Properties
        
        public int ServerPort => _serverPort;
        
        #endregion

        #region IInitializable Implementation
        
        public InitializationPhase Phase => InitializationPhase.PostInit;
        
        public void Initialize()
        {
            // Only initialize in data generation mode
            if (SimulationManager.Instance.CurrentMode != SimulationMode.DataGeneration)
            {
                Debug.Log("[DataGenerationController] Skipping initialization (not in data generation mode)");
                return;
            }
            
            Debug.Log($"[DataGenerationController] Initializing data generation server on port {_serverPort}...");
            
            if (_autoStartServer)
            {
                StartServer();
            }
        }
        
        public void Shutdown()
        {
            Debug.Log("[DataGenerationController] Stopping data generation server...");
            StopServer();
        }
        
        #endregion
        
        #region Public Methods
        
        /// <summary>
        /// Starts the TCP server for Python orchestrator connections.
        /// </summary>
        public void StartServer()
        {
            Debug.Log($"[DataGenerationController] Starting server on port {_serverPort}...");
            // TODO: Implement TCP server
        }
        
        /// <summary>
        /// Stops the TCP server.
        /// </summary>
        public void StopServer()
        {
            Debug.Log("[DataGenerationController] Stopping server...");
            // TODO: Clean up TCP server
        }
        
        #endregion
    }
}
