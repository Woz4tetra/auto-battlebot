// Auto-Battlebot Simulation System
// Initialization phase enumeration for ordered component startup

namespace AutoBattlebot.Core
{
    /// <summary>
    /// Defines the phases of initialization for simulation components.
    /// Components are initialized in phase order (PreInit -> Init -> PostInit -> Ready).
    /// </summary>
    public enum InitializationPhase
    {
        /// <summary>
        /// Pre-initialization phase. Used for components that must initialize
        /// before all others (e.g., configuration loading, logging setup).
        /// </summary>
        PreInit = 0,

        /// <summary>
        /// Main initialization phase. Used for core system components
        /// (e.g., communication bridge, camera system).
        /// </summary>
        Init = 1,

        /// <summary>
        /// Post-initialization phase. Used for components that depend on
        /// core systems being ready (e.g., robot spawning, AI setup).
        /// </summary>
        PostInit = 2,

        /// <summary>
        /// Final ready phase. Used for components that need all other
        /// systems initialized before they can start (e.g., simulation start trigger).
        /// </summary>
        Ready = 3
    }
}
