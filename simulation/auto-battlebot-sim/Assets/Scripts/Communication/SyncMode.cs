// Auto-Battlebot Simulation System
// Synchronization mode enumeration

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Defines the synchronization mode between Unity and C++.
    /// </summary>
    public enum SyncMode
    {
        /// <summary>
        /// Lockstep mode: Unity waits for C++ acknowledgment before proceeding.
        /// Ensures deterministic frame-by-frame execution.
        /// Use for: debugging, deterministic replays, slow-motion analysis.
        /// </summary>
        Lockstep = 0,
        
        /// <summary>
        /// Free-running mode: Unity sends frames without waiting for acknowledgment.
        /// C++ processes frames as fast as it can, may skip frames.
        /// Use for: real-time performance testing, stress testing.
        /// </summary>
        FreeRunning = 1
    }
}
