// Auto-Battlebot Simulation System
// Core initialization interface for ordered component startup

namespace AutoBattlebot.Core
{
    /// <summary>
    /// Interface for components that require ordered initialization.
    /// Components implementing this interface will be initialized by the SimulationManager
    /// in the order specified by their InitializationPhase.
    /// </summary>
    public interface IInitializable
    {
        /// <summary>
        /// The phase during which this component should be initialized.
        /// </summary>
        InitializationPhase Phase { get; }

        /// <summary>
        /// Called by SimulationManager during the appropriate initialization phase.
        /// </summary>
        void Initialize();

        /// <summary>
        /// Called when the simulation is shutting down.
        /// Use this to clean up resources.
        /// </summary>
        void Shutdown();
    }
}
