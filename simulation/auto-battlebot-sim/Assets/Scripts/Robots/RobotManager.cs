// Auto-Battlebot Simulation System
// Robot spawning and lifecycle management

using System.Collections.Generic;
using UnityEngine;
using AutoBattlebot.Core;

namespace AutoBattlebot.Robots
{
    /// <summary>
    /// Manages robot instantiation, lifecycle, and provides access
    /// to robot references for other systems.
    /// 
    /// Script Execution Order: 0 (default)
    /// </summary>
    public class RobotManager : MonoBehaviour, IInitializable
    {
        #region Singleton
        
        private static RobotManager _instance;
        public static RobotManager Instance => _instance;
        
        #endregion

        #region Private Fields
        
        private readonly List<GameObject> _robots = new List<GameObject>();
        
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
            Debug.Log("[RobotManager] Initializing robot manager...");
            // TODO: Spawn robots based on configuration
            Debug.Log("[RobotManager] Robot manager ready");
        }
        
        public void Shutdown()
        {
            Debug.Log("[RobotManager] Destroying all robots...");
            foreach (var robot in _robots)
            {
                if (robot != null)
                {
                    Destroy(robot);
                }
            }
            _robots.Clear();
        }
        
        #endregion
        
        #region Public Methods
        
        /// <summary>
        /// Gets all active robots in the scene.
        /// </summary>
        public IReadOnlyList<GameObject> GetAllRobots() => _robots.AsReadOnly();
        
        #endregion
    }
}
