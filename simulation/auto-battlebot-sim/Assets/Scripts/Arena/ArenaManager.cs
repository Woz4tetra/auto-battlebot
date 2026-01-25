// Auto-Battlebot Simulation System
// Arena environment and lighting management

using UnityEngine;
using AutoBattlebot.Core;

namespace AutoBattlebot.Arena
{
    /// <summary>
    /// Manages the arena environment including lighting, boundaries,
    /// and physics configuration.
    /// 
    /// Script Execution Order: 0 (default)
    /// </summary>
    public class ArenaManager : MonoBehaviour, IInitializable
    {
        #region Singleton
        
        private static ArenaManager _instance;
        public static ArenaManager Instance => _instance;
        
        #endregion

        #region Serialized Fields
        
        [Header("Arena Bounds")]
        [SerializeField]
        [Tooltip("The arena boundary collider")]
        private Collider _arenaBounds;
        
        #endregion

        #region Properties
        
        /// <summary>
        /// Gets the bounds of the arena.
        /// </summary>
        public Bounds ArenaBounds => _arenaBounds != null ? _arenaBounds.bounds : new Bounds();
        
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
        
        public InitializationPhase Phase => InitializationPhase.Init;
        
        public void Initialize()
        {
            Debug.Log("[ArenaManager] Initializing arena...");
            // TODO: Configure arena lighting and physics
            Debug.Log("[ArenaManager] Arena ready");
        }
        
        public void Shutdown()
        {
            Debug.Log("[ArenaManager] Arena shutdown");
        }
        
        #endregion
    }
}
