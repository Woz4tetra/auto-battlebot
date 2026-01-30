// Auto-Battlebot Simulation System
// Simulation mode enumeration

namespace AutoBattlebot.Core
{
    /// <summary>
    /// Defines the operating mode of the simulation.
    /// </summary>
    public enum RobotSimulationMode
    {
        /// <summary>
        /// Standalone mode - simulation runs independently without external connections.
        /// Useful for testing and development.
        /// </summary>
        Standalone = 0,

        /// <summary>
        /// Hardware-in-the-Loop mode - simulation connects to the C++ application
        /// via shared memory IPC for end-to-end testing.
        /// </summary>
        HardwareInLoop = 1,

        /// <summary>
        /// Data generation mode - simulation is controlled by Python scripts
        /// via TCP for synthetic training data generation.
        /// </summary>
        DataGeneration = 2
    }
}
