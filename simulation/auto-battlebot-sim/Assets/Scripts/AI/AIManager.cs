// Auto-Battlebot Simulation System
// AI behavior management for autonomous robots

using UnityEngine;
using AutoBattlebot.Core;

namespace AutoBattlebot.AI
{
    /// <summary>
    /// Manages AI behaviors for autonomous robots (opponents and neutrals).
    /// 
    /// Script Execution Order: 200
    /// </summary>
    [DefaultExecutionOrder(200)]
    public class AIManager : MonoBehaviour, IInitializable
    {
        #region Singleton
        
        private static AIManager _instance;
        public static AIManager Instance => _instance;
        
        #endregion

        #region Unity Lifecycle
        
        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }
            _instance = this;
        }
        
        #endregion

        #region IInitializable Implementation
        
        public InitializationPhase Phase => InitializationPhase.PostInit;
        
        public void Initialize()
        {
            Debug.Log("[AIManager] Initializing AI manager...");
            // TODO: Set up AI controllers for autonomous robots
            Debug.Log("[AIManager] AI manager ready");
        }
        
        public void Shutdown()
        {
            Debug.Log("[AIManager] Shutting down AI systems...");
        }
        
        #endregion
    }
}
