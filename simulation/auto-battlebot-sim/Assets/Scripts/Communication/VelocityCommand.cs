// Auto-Battlebot Simulation System
// Velocity command structure for shared memory communication

using System.Runtime.InteropServices;

namespace AutoBattlebot.Communication
{
    /// <summary>
    /// Velocity command structure received from the C++ application.
    /// Total size: 32 bytes (fixed-size for efficient IPC)
    /// Ensure the data layout matches include/shared_memory/simulation/simulation_velocity_command.hpp
    /// 
    /// This struct must match the C++ side exactly for binary compatibility.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VelocityCommand
    {
        /// <summary>
        /// Incrementing command identifier for synchronization.
        /// Unity uses this to detect new commands from C++.
        /// </summary>
        public ulong CommandId;          // 8 bytes (offset 0)

        /// <summary>
        /// Linear velocity in X direction (forward/backward) in m/s.
        /// Positive = forward, Negative = backward.
        /// </summary>
        public double LinearX;           // 8 bytes (offset 8)

        /// <summary>
        /// Linear velocity in Y direction (strafe left/right) in m/s.
        /// Positive = right, Negative = left.
        /// </summary>
        public double LinearY;           // 8 bytes (offset 16)

        /// <summary>
        /// Angular velocity around Z axis (rotation) in rad/s.
        /// Positive = counter-clockwise, Negative = clockwise.
        /// </summary>
        public double AngularZ;          // 8 bytes (offset 24)

        /// <summary>
        /// Total command size in bytes.
        /// </summary>
        public const int SIZE = 32;

        /// <summary>
        /// Default maximum linear velocity in m/s for validation.
        /// </summary>
        public const double DEFAULT_MAX_LINEAR_VELOCITY = 10.0;

        /// <summary>
        /// Default maximum angular velocity in rad/s for validation.
        /// </summary>
        public const double DEFAULT_MAX_ANGULAR_VELOCITY = 20.0;

        /// <summary>
        /// Creates a zero velocity command.
        /// </summary>
        public static VelocityCommand Zero => new VelocityCommand
        {
            CommandId = 0,
            LinearX = 0.0,
            LinearY = 0.0,
            AngularZ = 0.0
        };

        /// <summary>
        /// Validates that command values are within acceptable ranges.
        /// </summary>
        /// <param name="maxLinearVelocity">Maximum allowed linear velocity in m/s.</param>
        /// <param name="maxAngularVelocity">Maximum allowed angular velocity in rad/s.</param>
        /// <returns>True if all values are within range.</returns>
        public bool IsValid(double maxLinearVelocity = DEFAULT_MAX_LINEAR_VELOCITY,
                           double maxAngularVelocity = DEFAULT_MAX_ANGULAR_VELOCITY)
        {
            // Check for NaN or Infinity
            if (double.IsNaN(LinearX) || double.IsInfinity(LinearX) ||
                double.IsNaN(LinearY) || double.IsInfinity(LinearY) ||
                double.IsNaN(AngularZ) || double.IsInfinity(AngularZ))
            {
                return false;
            }

            // Check velocity ranges
            if (System.Math.Abs(LinearX) > maxLinearVelocity ||
                System.Math.Abs(LinearY) > maxLinearVelocity ||
                System.Math.Abs(AngularZ) > maxAngularVelocity)
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Returns a string representation of the command.
        /// </summary>
        public override string ToString()
        {
            return $"VelocityCommand(id={CommandId}, linear=({LinearX:F3}, {LinearY:F3}), angular={AngularZ:F3})";
        }
    }
}
