// Auto-Battlebot Simulation System
// Configuration loading and management

using UnityEngine;
using AutoBattlebot.Core;

namespace AutoBattlebot.Configuration
{
    /// <summary>
    /// Loads and manages simulation configuration from TOML files.
    /// 
    /// Script Execution Order: -500
    /// </summary>
    [DefaultExecutionOrder(-500)]
    public class ConfigurationLoader : MonoBehaviour, IInitializable
    {
        #region Serialized Fields
        
        [Header("Configuration")]
        [SerializeField]
        [Tooltip("Path to the main configuration file (relative to StreamingAssets)")]
        private string _configPath = "config/simulation.toml";
        
        #endregion

        #region IInitializable Implementation
        
        public InitializationPhase Phase => InitializationPhase.PreInit;
        
        public void Initialize()
        {
            Debug.Log("[ConfigurationLoader] Loading configuration...");
            // TODO: Implement TOML configuration loading
            Debug.Log("[ConfigurationLoader] Configuration loaded");
        }
        
        public void Shutdown()
        {
            Debug.Log("[ConfigurationLoader] Shutdown");
        }
        
        #endregion
    }
}
